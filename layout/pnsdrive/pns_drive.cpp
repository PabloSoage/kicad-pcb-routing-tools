// pns_drive: KiCad's interactive router (PNS) driven by a text script, without a GUI.
// Built inside the KiCad 10.0.8 source tree (qa/tools/pns), linked like qa_pns_regressions.
// GPL-3.0-or-later, as the KiCad code it links.
//
// Script, one command per line, coordinates in mm, "#" starts a comment:
//   load BOARD.kicad_pcb            project (.kicad_pro) and rules (.kicad_dru) beside it
//   save OUT.kicad_pcb
//   policy shove|walk|mark          collision policy (default shove)
//   mode single|dp                  routing mode
//   start X Y LAYER [NET]           begin a route at the item under X,Y (pad, via or track end)
//   move X Y                        drag the head to X,Y
//   layer LAYER                     switch layer (places a via at the next fix)
//   via                             toggle via placement
//   fix X Y                         fix up to X,Y (finishes at a pad/track end)
//   finish X Y                      fix with force-finish
//   commit / abort                  commit or drop the route in progress
//   amp MIN MAX SPACING             meander shape (mm)
//   tune dp|single|skew X1 Y1 X2 Y2 LAYER TARGET [NET]
//                                   meander between the two points; TARGET in mm (length)
//                                   or ps (skew)
//   drag X Y LAYER X2 Y2 [NET]      drag the segment/via under X,Y to X2,Y2 (shoves others)
//   length NET                      print the routed length of NET's tracks
//   size W GAP                      override track width and pair gap (mm); "size 0 0" = rules
// Each command prints "ok ..." or "FAIL ...". Exit code 1 if any command failed.

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <wx/init.h>
#include <wx/app.h>
#include <kiplatform/app.h>
#include <mock_pgm_base.h>
#include <pcbnew_settings.h>
#include <wx/filename.h>
#include <board.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <pad.h>
#include <pcb_track.h>
#include <netinfo.h>
#include <project.h>
#include <settings/settings_manager.h>
#include <drc/drc_engine.h>
#include <pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h>
#include <reporter.h>
#include <router/pns_kicad_iface.h>
#include <router/pns_router.h>
#include <router/pns_routing_settings.h>
#include <router/pns_sizes_settings.h>
#include <router/pns_meander_placer_base.h>
#include <router/pns_item.h>
#include <router/pns_itemset.h>
#include <router/pns_solid.h>
#include <router/pns_segment.h>
#include <router/pns_arc.h>
#include <router/pns_via.h>
#include <router/pns_node.h>
#include <router/pns_debug_decorator.h>

static int mm( double v ) { return KiROUND( v * 1e6 ); }

class DRIVE_IFACE : public PNS_KICAD_IFACE_BASE
{
public:
    std::vector<BOARD_ITEM*> m_graveyard;
    int                      m_added = 0, m_removed = 0;

    BOARD_CONNECTED_ITEM* create( PNS::ITEM* aItem )
    {
        NETINFO_ITEM* net = static_cast<NETINFO_ITEM*>( aItem->Net() );

        if( !net )
            net = NETINFO_LIST::OrphanedItem();

        switch( aItem->Kind() )
        {
        case PNS::ITEM::ARC_T:
        {
            PNS::ARC* arc = static_cast<PNS::ARC*>( aItem );
            PCB_ARC*  a = new PCB_ARC( m_board, static_cast<const SHAPE_ARC*>( arc->Shape( -1 ) ) );
            a->SetWidth( arc->Width() );
            a->SetLayer( GetBoardLayerFromPNSLayer( arc->Layers().Start() ) );
            a->SetNet( net );
            return a;
        }
        case PNS::ITEM::SEGMENT_T:
        {
            PNS::SEGMENT* seg = static_cast<PNS::SEGMENT*>( aItem );
            PCB_TRACK*    t = new PCB_TRACK( m_board );
            t->SetStart( seg->Seg().A );
            t->SetEnd( seg->Seg().B );
            t->SetWidth( seg->Width() );
            t->SetLayer( GetBoardLayerFromPNSLayer( seg->Layers().Start() ) );
            t->SetNet( net );
            return t;
        }
        case PNS::ITEM::VIA_T:
        {
            PNS::VIA* via = static_cast<PNS::VIA*>( aItem );
            PCB_VIA*  v = new PCB_VIA( m_board );
            v->SetPosition( via->Pos() );
            v->SetWidth( PADSTACK::ALL_LAYERS, via->Diameter( 0 ) );
            v->SetDrill( via->Drill() );
            v->SetNet( net );
            v->SetViaType( via->ViaType() );
            v->Padstack().SetUnconnectedLayerMode( via->UnconnectedLayerMode() );
            v->SetIsFree( via->IsFree() );
            v->SetLayerPair( GetBoardLayerFromPNSLayer( via->Layers().Start() ),
                             GetBoardLayerFromPNSLayer( via->Layers().End() ) );
            return v;
        }
        default: return nullptr;
        }
    }

    void AddItem( PNS::ITEM* aItem ) override
    {
        if( BOARD_CONNECTED_ITEM* bi = create( aItem ) )
        {
            aItem->SetParent( bi );
            m_board->Add( bi );
            m_added++;
        }
    }

    void RemoveItem( PNS::ITEM* aItem ) override
    {
        BOARD_ITEM* p = aItem->Parent();

        if( !p || aItem->OfKind( PNS::ITEM::SOLID_T ) )
            return;

        m_board->Remove( p );
        m_graveyard.push_back( p );
        m_removed++;
    }

    void UpdateItem( PNS::ITEM* aItem ) override
    {
        BOARD_ITEM* p = aItem->Parent();

        if( !p )
            return;

        if( aItem->Kind() == PNS::ITEM::SEGMENT_T )
        {
            PNS::SEGMENT* s = static_cast<PNS::SEGMENT*>( aItem );
            PCB_TRACK*    t = static_cast<PCB_TRACK*>( p );
            t->SetStart( s->Seg().A );
            t->SetEnd( s->Seg().B );
            t->SetWidth( s->Width() );
        }
        else if( aItem->Kind() == PNS::ITEM::VIA_T )
        {
            static_cast<PCB_VIA*>( p )->SetPosition( static_cast<PNS::VIA*>( aItem )->Pos() );
        }
    }
};


struct DRIVER
{
    std::unique_ptr<SETTINGS_MANAGER>      settings;
    std::unique_ptr<BOARD>                 board;
    std::unique_ptr<DRIVE_IFACE>           iface;
    std::unique_ptr<PNS::ROUTER>           router;
    std::unique_ptr<PNS::ROUTING_SETTINGS> rs;
    PNS::MEANDER_SETTINGS                  meander;
    PNS::PNS_MODE                          policy = PNS::RM_Shove;
    PNS::ROUTER_MODE                       mode = PNS::PNS_MODE_ROUTE_SINGLE;
    int                                    failures = 0;
    int                                    forceWidth = 0, forceGap = 0;

    bool fail( const std::string& msg )
    {
        printf( "FAIL %s\n", msg.c_str() );
        failures++;
        return false;
    }

    static std::string why( PNS::ROUTER* r ) { return std::string( r->FailureReason().ToUTF8() ); }

    bool load( const std::string& path )
    {
        wxFileName fn( wxString::FromUTF8( path ) );
        fn.MakeAbsolute();
        wxFileName pro( fn );
        pro.SetExt( "kicad_pro" );
        wxFileName dru( fn );
        dru.SetExt( "kicad_dru" );

        SETTINGS_MANAGER& mgr = Pgm().GetSettingsManager();
        mgr.LoadProject( pro.GetFullPath() );
        PROJECT* project = mgr.GetProject( pro.GetFullPath() );

        PCB_IO_KICAD_SEXPR io;
        board.reset( io.LoadBoard( fn.GetFullPath(), nullptr, nullptr ) );
        board->SetProject( project );

        std::shared_ptr<DRC_ENGINE> drc( new DRC_ENGINE );
        BOARD_DESIGN_SETTINGS&      bds = board->GetDesignSettings();
        bds.m_DRCEngine = drc;
        board->SynchronizeNetsAndNetClasses( true );
        drc->SetBoard( board.get() );
        drc->SetDesignSettings( &bds );
        drc->SetLogReporter( &NULL_REPORTER::GetInstance() );
        drc->InitEngine( dru.FileExists() ? dru : wxFileName() );

        board->SynchronizeTuningProfileProperties();
        resetRouter();
        printf( "ok loaded: %zu tracks, %zu footprints, rules %s\n", board->Tracks().size(),
                board->Footprints().size(), dru.FileExists() ? "yes" : "no" );
        return true;
    }

    void resetRouter()
    {
        iface.reset( new DRIVE_IFACE );
        iface->SetBoard( board.get() );
        iface->SetDebugDecorator( new PNS::DEBUG_DECORATOR );   // no-op; the iface owns it
        router.reset( new PNS::ROUTER );
        router->SetInterface( iface.get() );
        router->ClearWorld();
        router->SyncWorld();
        rs.reset( new PNS::ROUTING_SETTINGS( nullptr, "" ) );
        rs->SetMode( policy );
        rs->SetOptimizerEffort( PNS::OE_FULL );
        rs->SetRemoveLoops( true );
        rs->SetSmartPads( true );
        rs->SetSuggestFinish( false );
        rs->SetAllowDRCViolations( false );
        rs->SetFreeAngleMode( false );
        rs->SetFixAllSegments( true );
        rs->SetShoveVias( false );   // a via whose tracks end off its centre would be left behind
        router->LoadSettings( rs.get() );
    }

    void resync()
    {
        router->ClearWorld();
        router->SyncWorld();
    }

    int pnsLayer( const std::string& name )
    {
        PCB_LAYER_ID id = board->GetLayerID( wxString::FromUTF8( name ) );

        if( id == UNDEFINED_LAYER )
            return -1;

        return iface->GetPNSLayerFromBoardLayer( id );
    }

    static std::string netName( const PNS::ITEM* it )
    {
        const NETINFO_ITEM* n = static_cast<const NETINFO_ITEM*>( it->Net() );
        return n ? std::string( n->GetNetname().ToUTF8() ) : std::string();
    }

    static bool netMatch( const std::string& n, const std::string& want )
    {
        if( want.empty() || n == want )
            return true;

        // match on the last path element: "LINK_RX_D0_P" matches "/LINK_RX_D0_P"
        size_t s = n.rfind( '/' );
        return s != std::string::npos && n.substr( s + 1 ) == want;
    }

    // The item under P on LAYER, of NET if given; pads first, then vias, then tracks.
    PNS::ITEM* pick( const VECTOR2I& p, int layer, const std::string& net, int kinds )
    {
        for( int slop : { 0, mm( 0.05 ), mm( 0.15 ) } )
        {
            PNS::ITEM* best = nullptr;
            int        bestScore = -1;
            PNS::ITEM_SET set = router->QueryHoverItems( p, slop );

            for( PNS::ITEM* it : set.Items() )
            {
                if( !it->OfKind( kinds ) )
                    continue;

                if( layer >= 0 && !it->Layers().Overlaps( layer ) )
                    continue;

                if( !netMatch( netName( it ), net ) )
                    continue;

                int score = it->OfKind( PNS::ITEM::SOLID_T ) ? 3
                          : it->OfKind( PNS::ITEM::VIA_T )   ? 2 : 1;

                if( score > bestScore )
                {
                    best = it;
                    bestScore = score;
                }
            }

            if( best )
                return best;
        }

        return nullptr;
    }

    bool start( const VECTOR2I& p, int layer, const std::string& net )
    {
        PNS::ITEM* it = pick( p, layer, net, PNS::ITEM::ANY_T );

        if( !it )
            return fail( "start: nothing of that net under the point" );

        router->SetMode( mode );
        PNS::SIZES_SETTINGS sizes( router->Sizes() );
        iface->SetStartLayerFromPNS( layer );
        iface->ImportSizes( sizes, it, it->Net(), p );

        if( forceWidth > 0 )
        {
            sizes.SetTrackWidth( forceWidth );
            sizes.SetDiffPairWidth( forceWidth );
        }

        if( forceGap > 0 )
            sizes.SetDiffPairGap( forceGap );

        router->UpdateSizes( sizes );

        if( !router->StartRouting( p, it, layer ) )
            return fail( "start: " + why( router.get() ) );

        m_lastFix = p;
        m_routeNet = netName( it );

        printf( "ok start on %s, width %.4f gap %.4f via %.3f/%.3f\n", netName( it ).c_str(),
                sizes.TrackWidth() / 1e6, sizes.DiffPairGap() / 1e6, sizes.ViaDiameter() / 1e6,
                sizes.ViaDrill() / 1e6 );
        return true;
    }

    bool tune( const std::string& kind, const VECTOR2I& a, const VECTOR2I& b, int layer,
               double target, const std::string& net )
    {
        PNS::ITEM* it = pick( a, layer, net, PNS::ITEM::SEGMENT_T | PNS::ITEM::ARC_T );

        if( !it )
            return fail( "tune: no track under the first point" );

        PNS::ROUTER_MODE m = kind == "dp"     ? PNS::PNS_MODE_TUNE_DIFF_PAIR
                           : kind == "skew"   ? PNS::PNS_MODE_TUNE_DIFF_PAIR_SKEW
                                              : PNS::PNS_MODE_TUNE_SINGLE;
        router->SetMode( m );

        PNS::MEANDER_SETTINGS s = meander;
        s.m_isTimeDomain = false;
        s.m_overrideCustomRules = true;
        s.m_keepEndpoints = true;

        if( BOARD_CONNECTED_ITEM* bci = dynamic_cast<BOARD_CONNECTED_ITEM*>( it->Parent() ) )
            s.m_netClass = bci->GetEffectiveNetClass();

        if( kind == "skew" )
            s.SetTargetSkew( mm( target / 6.6 ) );   // ps -> mm at ~6.6 ps/mm (stripline)
        else
            s.SetTargetLength( (long long) mm( target ) );

        if( !router->StartRouting( a, it, layer ) )
            return fail( "tune start: " + why( router.get() ) );

        auto* placer = static_cast<PNS::MEANDER_PLACER_BASE*>( router->Placer() );
        placer->UpdateSettings( s );
        router->Move( b, nullptr );
        long long got = placer->TuningLengthResult();
        int       st = placer->TuningStatus();

        if( !router->FixRoute( b, nullptr, false, false ) )
            return fail( "tune fix" );

        router->StopRouting();
        resync();
        printf( "ok tune %s: result %.4f mm (%s)\n", kind.c_str(), got / 1e6,
                st == PNS::MEANDER_PLACER_BASE::TUNED       ? "tuned"
                : st == PNS::MEANDER_PLACER_BASE::TOO_SHORT ? "too short"
                                                            : "too long" );
        return true;
    }

    bool drag( const VECTOR2I& a, int layer, const VECTOR2I& b, const std::string& net )
    {
        PNS::ITEM* it = pick( a, layer, net,
                              PNS::ITEM::SEGMENT_T | PNS::ITEM::ARC_T | PNS::ITEM::VIA_T );

        if( !it )
            return fail( "drag: nothing under the point" );

        router->SetMode( mode );
        PNS::ITEM_SET set( it );
        int dm = it->OfKind( PNS::ITEM::VIA_T ) ? PNS::DM_VIA : PNS::DM_ANY;

        if( !router->StartDragging( a, set, dm ) )
            return fail( "drag start: " + why( router.get() ) );

        router->Move( b, nullptr );

        if( !router->FixRoute( b, nullptr, false, true ) )
        {
            router->StopRouting();
            resync();
            return fail( "drag fix (blocked)" );
        }

        router->StopRouting();
        resync();
        printf( "ok drag\n" );
        return true;
    }

    // Follow a polyline from the current head: for each point, fix at the furthest of
    // 100/75/50/25/12 % of the way that fits; repeat until the point is reached or nothing fits.
    VECTOR2I m_lastFix;

    bool follow( const std::vector<VECTOR2I>& pts )
    {
        for( const VECTOR2I& target : pts )
        {
            int guard = 0;

            while( ( target - m_lastFix ).EuclideanNorm() > mm( 0.01 ) && guard++ < 40 )
            {
                bool progressed = false;

                for( double f : { 1.0, 0.75, 0.5, 0.25, 0.12 } )
                {
                    VECTOR2I p = m_lastFix + ( target - m_lastFix ) * f;

                    // a mouse trail: the router picks the corner posture from it
                    VECTOR2I d = p - m_lastFix;
                    int      steps = std::max( 1, (int) ( d.EuclideanNorm() / mm( 0.1 ) ) );

                    for( int k = 1; k < steps; k++ )
                        router->Move( m_lastFix + d * k / steps, nullptr );

                    if( !router->Move( p, nullptr ) )
                        continue;

                    double before = netLength( m_routeNet );
                    router->FixRoute( p, nullptr, false, false );
                    double gained = netLength( m_routeNet ) - before;

                    // real progress: new copper of about the distance asked for (a fix can
                    // also just re-commit what was there)
                    if( gained >= 0.5 * ( p - m_lastFix ).EuclideanNorm() / 1e6 )
                    {
                        m_lastFix = p;
                        progressed = true;
                        break;
                    }
                }

                if( !progressed )
                    return fail( wxString::Format( "follow: stuck at %.3f %.3f heading for %.3f %.3f",
                                                   m_lastFix.x / 1e6, m_lastFix.y / 1e6,
                                                   target.x / 1e6, target.y / 1e6 ).ToStdString() );
            }
        }

        printf( "ok follow reached %.3f %.3f\n", m_lastFix.x / 1e6, m_lastFix.y / 1e6 );
        return true;
    }

    std::string m_routeNet;

    double netLength( const std::string& net )
    {
        double sum = 0;

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( t->Type() != PCB_VIA_T && netMatch( std::string( t->GetNetname().ToUTF8() ), net ) )
                sum += t->GetLength();
        }

        return sum / 1e6;
    }

    void length( const std::string& net )
    {
        double sum = 0;

        for( PCB_TRACK* t : board->Tracks() )
        {
            if( t->Type() != PCB_VIA_T && netMatch( std::string( t->GetNetname().ToUTF8() ), net ) )
                sum += t->GetLength();
        }

        printf( "ok length %s %.4f mm\n", net.c_str(), sum / 1e6 );
    }

    void run( std::istream& in )
    {
        std::string line;

        while( std::getline( in, line ) )
        {
            if( size_t h = line.find( '#' ); h != std::string::npos )
                line.resize( h );

            std::istringstream ss( line );
            std::string        cmd;

            if( !( ss >> cmd ) )
                continue;

            printf( "> %s\n", line.c_str() );
            fflush( stdout );

            double      x = 0, y = 0, x2 = 0, y2 = 0, t = 0;
            std::string a, b, net;

            if( cmd == "load" )
            {
                ss >> a;
                load( a );
            }
            else if( !board )
            {
                fail( "no board loaded" );
            }
            else if( cmd == "save" )
            {
                ss >> a;

                if( router->RoutingInProgress() )
                    router->StopRouting();

                PCB_IO_KICAD_SEXPR io;
                io.SaveBoard( wxString::FromUTF8( a ), board.get() );
                printf( "ok saved %s (+%d -%d items)\n", a.c_str(), iface->m_added,
                        iface->m_removed );
            }
            else if( cmd == "policy" )
            {
                ss >> a;
                policy = a == "walk"   ? PNS::RM_Walkaround
                       : a == "mark"   ? PNS::RM_MarkObstacles
                                       : PNS::RM_Shove;
                rs->SetMode( policy );
                printf( "ok\n" );
            }
            else if( cmd == "mode" )
            {
                ss >> a;
                mode = a == "dp" ? PNS::PNS_MODE_ROUTE_DIFF_PAIR : PNS::PNS_MODE_ROUTE_SINGLE;
                printf( "ok\n" );
            }
            else if( cmd == "start" )
            {
                ss >> x >> y >> a >> net;
                int l = pnsLayer( a );

                if( l < 0 )
                    fail( "unknown layer " + a );
                else
                    start( { mm( x ), mm( y ) }, l, net );
            }
            else if( cmd == "move" )
            {
                ss >> x >> y;

                bool     ok = router->Move( { mm( x ), mm( y ) }, nullptr );
                VECTOR2I e = router->Placer() ? router->Placer()->CurrentEnd() : VECTOR2I();
                double   d = ( e - VECTOR2I( mm( x ), mm( y ) ) ).EuclideanNorm() / 1e6;

                if( ok && d < 0.01 )
                    printf( "ok\n" );
                else
                    fail( wxString::Format( "move: head at %.3f %.3f, %.3f mm short (%s)",
                                            e.x / 1e6, e.y / 1e6, d, ok ? "ok" : "router false" )
                                  .ToStdString() );
            }
            else if( cmd == "layer" )
            {
                ss >> a;
                int l = pnsLayer( a );

                if( l < 0 || !router->SwitchLayer( l ) )
                    fail( "layer " + a );
                else
                    printf( "ok\n" );
            }
            else if( cmd == "via" )
            {
                router->ToggleViaPlacement();
                printf( "ok\n" );
            }
            else if( cmd == "fix" || cmd == "finish" )
            {
                ss >> x >> y;
                VECTOR2I p( mm( x ), mm( y ) );
                PNS::ITEM* end = pick( p, -1, "", PNS::ITEM::SOLID_T | PNS::ITEM::VIA_T
                                                          | PNS::ITEM::SEGMENT_T | PNS::ITEM::ARC_T );

                // the end item must reach Move(): that is where the placer snaps onto it
                VECTOR2I d = p - m_lastFix;
                int      steps = std::max( 1, (int) ( d.EuclideanNorm() / mm( 0.1 ) ) );

                for( int k = 1; k <= steps; k++ )
                    router->Move( m_lastFix + d * k / steps, k == steps ? end : nullptr );

                int  before = iface->m_added;
                bool ok = router->FixRoute( p, end, cmd == "finish", false );
                bool done = !router->RoutingInProgress();

                if( !ok && iface->m_added == before )
                    fail( cmd + ": nothing fixed (head collides) " + why( router.get() ) );
                else
                    m_lastFix = p, printf( "ok %s (+%d items)\n", done ? "route complete" : "fixed, continuing",
                            iface->m_added - before );

                if( done )
                    resync();
            }
            else if( cmd == "commit" )
            {
                if( router->RoutingInProgress() )
                {
                    router->CommitRouting();
                    router->StopRouting();
                }

                resync();
                printf( "ok\n" );
            }
            else if( cmd == "abort" )
            {
                router->StopRouting();
                resync();
                printf( "ok\n" );
            }
            else if( cmd == "amp" )
            {
                ss >> x >> y >> t;
                meander.m_minAmplitude = mm( x );
                meander.m_maxAmplitude = mm( y );
                meander.m_spacing = mm( t );
                printf( "ok\n" );
            }
            else if( cmd == "tune" )
            {
                ss >> a >> x >> y >> x2 >> y2 >> b >> t >> net;
                tune( a, { mm( x ), mm( y ) }, { mm( x2 ), mm( y2 ) }, pnsLayer( b ), t, net );
            }
            else if( cmd == "drag" )
            {
                ss >> x >> y >> a >> x2 >> y2 >> net;
                drag( { mm( x ), mm( y ) }, pnsLayer( a ), { mm( x2 ), mm( y2 ) }, net );
            }
            else if( cmd == "shovevias" )
            {
                ss >> a;
                rs->SetShoveVias( a == "on" );
                printf( "ok\n" );
            }
            else if( cmd == "violate" )
            {
                ss >> a;
                rs->SetAllowDRCViolations( a == "on" );
                printf( "ok\n" );
            }
            else if( cmd == "fixall" )
            {
                ss >> a;
                rs->SetFixAllSegments( a == "on" );
                printf( "ok\n" );
            }
            else if( cmd == "follow" )
            {
                std::vector<VECTOR2I> pts;

                while( ss >> x >> y )
                    pts.emplace_back( mm( x ), mm( y ) );

                follow( pts );
            }
            else if( cmd == "size" )
            {
                ss >> x >> y;
                forceWidth = mm( x );
                forceGap = mm( y );
                printf( "ok\n" );
            }
            else if( cmd == "length" )
            {
                ss >> a;
                length( a );
            }
            else
            {
                fail( "unknown command " + cmd );
            }

            fflush( stdout );
        }
    }
};


int main( int argc, char* argv[] )
{
    SetPgm( new MOCK_PGM_BASE() );
    KIPLATFORM::APP::Init();
    wxApp::SetInstance( new wxAppConsole );

    if( !wxInitialize( argc, argv ) )
        return 2;

    Pgm().InitPgm( true, true, true );
    Pgm().GetSettingsManager().RegisterSettings( new PCBNEW_SETTINGS, false );
    Pgm().GetSettingsManager().Load();

    DRIVER d;

    if( argc > 1 )
    {
        std::ifstream f( argv[1] );
        d.run( f );
    }
    else
    {
        d.run( std::cin );
    }

    return d.failures ? 1 : 0;
}
