"""Two-bolt keyed rail clamp, rigid camera frame; independent measured/design inputs."""
import json,math,os,sys,traceback
import cadquery as cq
from build_mount import ROOT,block,cylinder,rail,export_checked,descriptor

def xcyl(r,x0,length,y,z):
    return cq.Workplane(obj=cq.Solid.makeCylinder(r,length,cq.Vector(x0,y,z),cq.Vector(1,0,0)))

def place(part):
    return part.rotate((0,0,0),(1,0,0),-90).translate((20,54,30))

def run():
    cfg=json.loads((ROOT/'config/camera_attachment_v1.json').read_text())
    r=json.loads((ROOT/'config/mount_v0.json').read_text())['rail']
    out=ROOT/'output/camera_attachment_v1';out.mkdir(exist_ok=True,parents=True)
    h=r['height'];lo=h-r['top_to_lower_datum'];up=lo+r['datum_vertical_separation']
    w=r['max_width'];neck=r['neck_width'];datum=r['datum_width']
    cl=lo+(w-datum)/2;ch=up-(w-datum)/2;shoulder=lo-(datum-neck)/2;top=datum/2-(h-up)
    profile=[(-neck/2,-2),(neck/2,-2),(neck/2,shoulder),(w/2,cl),(w/2,ch),(top,h),(-top,h),(-w/2,ch),(-w/2,cl),(-neck/2,shoulder)]
    plane=cq.Plane(origin=(-1,0,0),xDir=(0,1,0),normal=(1,0,0))
    # Pure lateral allowance: closing 0.15 mm per side restores the exact rail
    # flank angles; isotropic offset would leave rocking clearance at the flanks.
    enlarged=[(y+math.copysign(cfg['rail_profile_clearance_mm'],y),z) for y,z in profile]
    cavity=cq.Workplane(plane).polyline(enlarged).close().extrude(35)
    # Fixed jaw + bridge; movable jaw has a 0.8 mm closing gap above the rail.
    main=block(32,27,15.6,(16,-3.5,16.2)).cut(cavity) # avoid inherited root edge break
    main=main.cut(block(34,5,6.15,(16,9.5,11.075)))
    jaw=block(32,8.2,15.6,(16,14.9,16.2)).union(block(32,11.5,5.75,(16,13.25,11.275))).cut(cavity)
    keys=[.5*r['slot_pitch'],2.5*r['slot_pitch']]
    for x in keys:
        key_bottom=h-cfg['index_key_depth_mm'];key_top=h+.45
        main=main.union(block(cfg['index_key_width_mm'],12,key_top-key_bottom,(x,0,(key_bottom+key_top)/2)))
    # Rigid frame, ahead of the rail, open rear for components and USB lead.
    frame=block(5,40,40,(39.5,0,24)).cut(block(7,22,22,(39.5,0,24)))
    frame=frame.cut(block(8,12,12,(39.5,0,41))) # upper central connector exit
    main=main.union(frame)
    # Twin tapered longitudinal webs carry loads into the clamp roof.
    for y in [-14,14]:
        pl=cq.Plane(origin=(0,y-2.5,0),xDir=(1,0,0),normal=(0,-1,0))
        # Plane local Y is +Z; extrude in -Y.
        pl=cq.Plane(origin=(0,y+2.5,0),xDir=(1,0,0),normal=(0,-1,0))
        zbase=23.8 if y<0 else 24.3
        web=cq.Workplane(pl).polyline([(16,zbase),(37.2,zbase),(37.2,41)]).close().extrude(5)
        main=main.union(web)
        main=main.union(block(9 if y<0 else 6.7,5,10,(34.5 if y<0 else 35.65,y,19)))
    # Four local bearing pads; allow diagonal adjustment without guessing pitch.
    for sy in [-1,1]:
        for sz in [-1,1]:
            cy=sy*14;cz=24+sz*14
            main=main.union(block(4.1,7.4,7.4,(44.05,cy,cz))) # pcb back x46.1
            p=cq.Plane(origin=(36.8,0,24),xDir=(0,1,0),normal=(1,0,0))
            angle=45 if sy==sz else -45
            slot=cq.Workplane(p).center(cy,sz*14).slot2D(2*math.sqrt(2)+cfg['camera_slot_diameter_mm'],cfg['camera_slot_diameter_mm'],angle).extrude(10)
            main=main.cut(slot)
            access_plane=cq.Plane(origin=(28,0,24),xDir=(0,1,0),normal=(1,0,0))
            access=cq.Workplane(access_plane).center(cy,sz*14).slot2D(2*math.sqrt(2)+6.2,6.2,angle).extrude(9)
            main=main.cut(access)
    # Edge datums at y +/-16.2; seat PCB against one side before tightening.
    for sy in [-1,1]:
        main=main.union(block(6,2,7,(44.9,sy*17.2,24)))
    # Axial take-up screw housing; accessible from front before PCB assembly.
    main=main.union(block(9,10,9,(40,0,12.5)))
    main=main.cut(xcyl(1.7,35,11,0,12.5))
    main=main.cut(xcyl(3.1,40.5,5,0,12.5))
    np=cq.Plane(origin=(36.6,0,12.5),xDir=(0,1,0),normal=(1,0,0))
    main=main.cut(cq.Workplane(np).polygon(6,5.7/math.cos(math.pi/6)).extrude(2.6))
    # Nut slot opens downward for installation; M3 metal nut, no tapped plastic.
    main=main.cut(block(2.6,5.7,5,(37.9,0,8.5)))
    # Cable tie slots in outboard side tab, away from board components.
    main=main.union(block(10,7,4,(27,-19,26)))
    for x in [24.5,29.5]:main=main.cut(block(2,4,6,(x,-19,26)))
    for x in keys:
        shaft=cylinder(2.25,-18,39,x,19)
        main=main.cut(shaft);jaw=jaw.cut(shaft)
        # M4 nut trapped in fixed side, head and washer on removable side.
        pl=cq.Plane(origin=(x,-17.1,19),xDir=(1,0,0),normal=(0,1,0))
        main=main.cut(cq.Workplane(pl).polygon(6,7.2/math.cos(math.pi/6)).extrude(3.6))
    main=main.clean();jaw=jaw.clean()
    parts={'camera_clamp_body':main,'camera_clamp_jaw':jaw}
    reports={k:export_checked(v,k,out) for k,v in parts.items()}
    actual=cq.importers.importStep(str(ROOT/'output/v3/G1_head_tri_rail_monolithic_v3.step'))
    for k,s in parts.items():
        overlap=place(s).intersect(actual).val().Volume();assert overlap<1e-5,(k,overlap)
        reports[k]['v3_mount_intersection_mm3']=overlap
    assert main.intersect(jaw).val().Volume()<1e-5
    # Verify intended side-contact closure before jaw/body bottoming.
    closed_main=main.translate((0,.15,0));closed_jaw=jaw.translate((0,-.15,0))
    assert place(closed_main).intersect(actual).val().Volume()<1e-5
    assert place(closed_jaw).intersect(actual).val().Volume()<1e-5
    assert closed_main.intersect(closed_jaw).val().Volume()<1e-5
    assert place(main.translate((0,.19,0))).intersect(actual).val().Volume()>.01
    assert place(jaw.translate((0,-.19,0))).intersect(actual).val().Volume()>.01
    # Both M4 shafts clear the clamp and the actual parent rail.
    for x in keys:
        probe=cylinder(2,-17,38,x,19)
        assert probe.intersect(main).val().Volume()<1e-5
        assert probe.intersect(jaw).val().Volume()<1e-5
        assert place(probe).intersect(actual).val().Volume()<1e-5
        # 40 mm shaft, 1 mm flat washer and head outside the movable jaw.
        hardware=cylinder(2,-20,40,x,19).union(cylinder(3.5,20,4,x,19))
        washer=cylinder(4.5,19,1,x,19).cut(cylinder(2.15,18.9,1.2,x,19))
        assert hardware.intersect(main).val().Volume()<1e-5
        assert hardware.intersect(jaw).val().Volume()<1e-5
        assert washer.intersect(main).val().Volume()<1e-5
        assert place(hardware.union(washer)).intersect(actual).val().Volume()<1e-5
    stop=xcyl(1.5,35,6,0,12.5).union(xcyl(2.75,41,3,0,12.5))
    assert stop.intersect(main).val().Volume()<1e-5
    assert place(stop).intersect(actual).val().Volume()<1e-5
    # M2 fastener clearance at the extreme and mid positions of the design range.
    for pitch in [26,28,30]:
        for sy in [-1,1]:
            for sz in [-1,1]:
                qy=sy*pitch/2;qz=24+sz*pitch/2
                shank=xcyl(1,32,15,qy,qz)
                washer=xcyl(2.5,36.5,.5,qy,qz).cut(xcyl(1.1,36.4,.7,qy,qz))
                assert shank.intersect(main).val().Volume()<1e-5
                assert washer.intersect(main).val().Volume()<1e-5
    # Rail slot tolerance and forward seating travel.
    free_x=r['slot_width']-cfg['index_key_width_mm'];assert free_x>0
    key_contact=free_x/2
    assert place(main.translate((key_contact-.01,0,0))).intersect(actual).val().Volume()<1e-5
    assert place(main.translate((key_contact+.04,0,0))).intersect(actual).val().Volume()>.01
    # Detachable jaw can lift off laterally, with no need to slide over rail end.
    assert place(jaw.translate((0,3,0))).intersect(actual).val().Volume()<1e-5
    asm=cq.Assembly(name='G1_camera_clamp_on_v3')
    asm.add(actual,name='existing_v3_mount',color=cq.Color(.24,.32,.36))
    asm.add(place(main),name='clamp_and_camera_frame',color=cq.Color(.1,.62,.68))
    asm.add(place(jaw),name='removable_jaw',color=cq.Color(.9,.57,.18))
    asm.export(str(out/'assembly_on_v3.step'))
    for k,s in parts.items():cq.exporters.export(place(s),str(out/(k+'_assembly_coordinates.stl')),tolerance=.035,angularTolerance=.08)
    report=dict(parts=reports,config=descriptor(ROOT/'config/camera_attachment_v1.json'),
        camera_source=descriptor(ROOT/'sources/U20CAM-1080P-S1_UserManual-v1.1.pdf'),
        rail_slot_total_axial_clearance_mm=free_x,axial_adjustment_to_forward_seat_mm=key_contact,
        modeled_camera_board_back_x_mm=46.1,print_release=False,camera_fit_verified=False,
        closure_geometry_pass=True,axial_stop_pass=True,fastener_clearance_pass=True,
        required_verification=['Actual square hole pitch within 26..30 mm','Actual PCB thickness, backside components and connector clearance','Print rail fit and preload/slip under real load'],
        stiffness_note='Geometric interference and positive-stop checks only; no contact/preload FEA or physical slip rating.')
    (out/'validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':
    code=0
    try:run()
    except BaseException:traceback.print_exc();code=1
    sys.stdout.flush();sys.stderr.flush();os._exit(code)
