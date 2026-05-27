# Recon Scanner Enclosure — Autodesk Fusion 360 Python Script
# Generated for: Recon-Platform-R2 handheld LIDAR scanner
# Components: RPi5 + ESP32 DevKit V1 + Half breadboard + 149×69×28.6 battery + LD14P LIDAR
#
# Usage:
#   1. Fusion 360 → Utilities → Scripts and Add-Ins (Shift+S)
#   2. Scripts tab → green "+" → "Create from existing script" → pick this file
#   3. Select the script → "Run"
#   4. The active design's ROOT component is populated with these bodies:
#        enclosure_body     — matte black, the shelled main box
#        lid                — dark grey, removable top plate
#        grip_body          — matte black, pistol grip (rounded-rect, 15° tilt)
#        trigger_guard      — matte black, swept 5mm tube
#        ref_battery_red    — translucent red (149×69×28.6)
#        ref_rpi5_green     — translucent green (85×56×17)
#        ref_esp32_blue     — translucent blue (55×28×12)
#
# Note: Fusion 360 Part Design mode forbids sub-components, so everything
# lives flat in the root component. If you create the document as an
# Assembly Design instead, you can manually move bodies into sub-components
# afterward.
#
# Edit the CONFIG dict at the top to retune any dimension; geometry follows.

import adsk.core
import adsk.fusion
import adsk.cam
import traceback
import math


# =============================================================================
# CONFIG — every numeric constant lives here. All values in MILLIMETRES.
# Fusion 360's internal unit is centimetres; the mm() helper converts.
# =============================================================================
CONFIG = {
    # ---- Print / material ----
    'wall_t':                3.0,    # PETG / PLA wall thickness
    'fillet_exterior':       2.0,    # Exterior edge radius (printability)
    'chamfer_battery_bay':   1.0,    # Eases battery insertion

    # ---- Body box CAVITY (interior, top open for lid) ----
    'cavity_w':             80.0,    # X (left ↔ right)
    'cavity_d':            165.0,    # Y (back ↔ front; +Y forward)
    'cavity_h':             95.0,    # Z (cavity height — top is open)

    # ---- Lid ----
    'lid_thickness':         3.0,
    'lidar_cable_hole_d':   14.0,
    'lidar_seat_d':         40.0,
    'lidar_seat_depth':      3.0,
    'lid_screw_d':           3.0,    # M3 clearance through-hole in lid
    'lid_screw_inset':      10.0,    # inset from each corner

    # ---- Mounting bosses (interior top corners of the box) ----
    'boss_od':               6.0,
    'boss_id':               2.5,    # M3 tap-drill
    'boss_h':                8.0,

    # ---- FRONT (+Y) face features ----
    'front_slot_w':         30.0,
    'front_slot_h':         20.0,

    # ---- BACK (-Y) face features (recessed access panel — not a through cut) ----
    'back_panel_w':         50.0,
    'back_panel_h':         40.0,
    'back_panel_depth':      1.0,

    # ---- RIGHT (+X) face features ----
    'right_usb_port_d':     12.0,
    'right_usb_port_z':     30.0,    # from exterior bottom
    'right_switch_w':       10.0,
    'right_switch_h':        6.0,
    'right_switch_y':       30.0,
    'right_switch_z':       50.0,

    # ---- LEFT (-X) face features — three buttons in a vertical column ----
    'left_btn_d':           12.0,
    'left_btn_y':           40.0,
    'left_btn_z':         (25.0, 45.0, 65.0),

    # ---- GRIP ----
    'grip_len':            125.0,
    'grip_angle_deg':       15.0,    # back-tilt from -Z toward -Y
    'grip_cross_w':         28.0,    # X dimension of grip cross-section
    'grip_cross_d':         35.0,    # Y dimension of grip cross-section
    'grip_corner_r':         8.0,
    'grip_wall_t':           3.0,    # hollow grip wall thickness

    # ---- Trigger guard (swept circular profile) ----
    # Anchored at the grip's front-top edge (not the box's front-bottom
    # corner — that's 165mm away from the grip and looks wrong for a pistol).
    'trigger_tube_d':        4.0,    # 4mm tube — leaner than the original 5mm
    'trigger_fwd':          18.0,    # arc extends this far in +Y past the grip
    'trigger_down':         25.0,    # arc extends this far in -Z below box bottom

    # ---- Knurling pattern on grip (simplified to dome-bumps) ----
    'knurl_bump_d':          2.0,
    'knurl_bump_h':          0.8,
    'knurl_cols':              5,
    'knurl_rows':              8,
    'knurl_pitch':           5.0,

    # ---- Reference-only components (for visualising fit) ----
    'batt_w':              149.0,    # along Y (long axis = device depth)
    'batt_d':               69.0,    # along X
    'batt_h':               28.6,    # along Z

    'rpi_y':                85.0,    # along Y
    'rpi_x':                56.0,    # along X
    'rpi_z':                17.0,

    'esp_y':                55.0,    # along Y
    'esp_x':                28.0,    # along X
    'esp_z':                12.0,
}


# =============================================================================
# Helpers
# =============================================================================

def mm(v):
    """Convert millimetres → centimetres (Fusion 360 internal unit)."""
    return v / 10.0


def pt(x, y, z):
    """Point3D shortcut — coordinates already in CM."""
    return adsk.core.Point3D.create(x, y, z)


def vec(x, y, z):
    return adsk.core.Vector3D.create(x, y, z)


def vi(v):
    """ValueInput.createByReal shortcut."""
    return adsk.core.ValueInput.createByReal(v)


def oc(items):
    """ObjectCollection from a Python iterable."""
    col = adsk.core.ObjectCollection.create()
    for item in items:
        col.add(item)
    return col


def find_face_at_z(body, z, tol=1e-3):
    """Return the planar face on `body` whose normal is +Z and origin Z matches."""
    for face in body.faces:
        geom = face.geometry
        if not isinstance(geom, adsk.core.Plane):
            continue
        n = geom.normal
        if abs(n.x) < tol and abs(n.y) < tol and n.z > 0.9:
            if abs(geom.origin.z - z) < tol:
                return face
    return None


def group_timeline(timeline, start_marker, name):
    end = timeline.markerPosition - 1
    if end >= start_marker:
        try:
            grp = timeline.timelineGroups.add(start_marker, end)
            grp.name = name
        except Exception:
            pass


# =============================================================================
# Geometry construction
# =============================================================================

def make_body_box(comp):
    """Extrude the outer box solid, then shell off the +Z (top) face."""
    wall_t = mm(CONFIG['wall_t'])
    cav_w  = mm(CONFIG['cavity_w'])
    cav_d  = mm(CONFIG['cavity_d'])
    cav_h  = mm(CONFIG['cavity_h'])
    ext_w  = cav_w + 2 * wall_t
    ext_d  = cav_d + 2 * wall_t
    ext_h  = cav_h + wall_t           # bottom wall only — top is open for the lid

    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        pt(-ext_w / 2, -ext_d / 2, 0),
        pt( ext_w / 2,  ext_d / 2, 0))
    profile = sk.profiles.item(0)

    ein = comp.features.extrudeFeatures.createInput(
        profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ein.setDistanceExtent(False, vi(ext_h))
    feat = comp.features.extrudeFeatures.add(ein)
    body = feat.bodies.item(0)
    body.name = 'enclosure_body'

    # Shell — remove the +Z face inward by wall_t.
    top_face = find_face_at_z(body, ext_h)
    if top_face is None:
        raise RuntimeError('Could not find body top face for shell op')
    sh_in = comp.features.shellFeatures.createInput(oc([top_face]), False)
    sh_in.insideThickness = vi(wall_t)
    comp.features.shellFeatures.add(sh_in)
    return body, ext_w, ext_d, ext_h


def _add_cut_extrude(comp, prof, depth, through):
    """Robust cut: through-wall cuts use a symmetric extrude (total depth 2x
    `depth`, crosses the wall regardless of which way the construction plane's
    normal points). Partial cuts try both sign directions and keep whichever
    Fusion accepts. Either path swallows the 'no target body' error rather
    than aborting the whole script."""
    if through:
        # Symmetric distance — extrudes `depth` on EACH side of the plane,
        # so total cut depth = 2*depth. For wall cuts pass wall_t; the
        # 2*wall_t cut extends into air on one side and through the wall
        # on the other (air-side is a no-op).
        try:
            cin = comp.features.extrudeFeatures.createInput(
                prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
            cin.setDistanceExtent(True, vi(depth))
            comp.features.extrudeFeatures.add(cin)
        except Exception:
            pass
        return
    # Partial cut — try both directions; success on either is enough.
    for sign in (-1, 1):
        try:
            cin = comp.features.extrudeFeatures.createInput(
                prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
            cin.setDistanceExtent(False, vi(sign * depth))
            comp.features.extrudeFeatures.add(cin)
            return
        except Exception:
            continue


def cut_rect_on_plane(comp, plane, cx, cy, w, h, depth, through=False):
    """Sketch a centred rectangle on `plane` and cut."""
    sk = comp.sketches.add(plane)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        pt(cx - w / 2, cy - h / 2, 0),
        pt(cx + w / 2, cy + h / 2, 0))
    prof = sk.profiles.item(0)
    _add_cut_extrude(comp, prof, depth, through)


def cut_circle_on_plane(comp, plane, cx, cy, dia, depth, through=False):
    sk = comp.sketches.add(plane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(pt(cx, cy, 0), dia / 2)
    prof = sk.profiles.item(0)
    _add_cut_extrude(comp, prof, depth, through)


def cut_face_features(comp, ext_w, ext_d, ext_h):
    """All four side-face cuts: front slot, back recess, right ports, left btns."""
    wall_t = mm(CONFIG['wall_t'])

    # --- FRONT (+Y) slot — THROUGH the wall ---------------------------------
    p_in = comp.constructionPlanes.createInput()
    p_in.setByOffset(comp.xZConstructionPlane, vi(ext_d / 2))
    p_front = comp.constructionPlanes.add(p_in)
    cut_rect_on_plane(
        comp, p_front,
        cx=0, cy=ext_h / 2,
        w=mm(CONFIG['front_slot_w']),
        h=mm(CONFIG['front_slot_h']),
        depth=wall_t, through=True,
    )

    # --- BACK (-Y) recessed access panel (partial — try both directions) ----
    p_in = comp.constructionPlanes.createInput()
    p_in.setByOffset(comp.xZConstructionPlane, vi(-ext_d / 2))
    p_back = comp.constructionPlanes.add(p_in)
    cut_rect_on_plane(
        comp, p_back,
        cx=0, cy=ext_h / 2,
        w=mm(CONFIG['back_panel_w']),
        h=mm(CONFIG['back_panel_h']),
        depth=mm(CONFIG['back_panel_depth']),
    )

    # --- RIGHT (+X) face: 12mm USB port + 10x6 switch slot (THROUGH) --------
    p_in = comp.constructionPlanes.createInput()
    p_in.setByOffset(comp.yZConstructionPlane, vi(ext_w / 2))
    p_right = comp.constructionPlanes.add(p_in)
    cut_circle_on_plane(
        comp, p_right,
        cx=0, cy=mm(CONFIG['right_usb_port_z']),
        dia=mm(CONFIG['right_usb_port_d']),
        depth=wall_t, through=True,
    )
    cut_rect_on_plane(
        comp, p_right,
        cx=mm(CONFIG['right_switch_y']) - ext_d / 2,
        cy=mm(CONFIG['right_switch_z']),
        w=mm(CONFIG['right_switch_w']),
        h=mm(CONFIG['right_switch_h']),
        depth=wall_t, through=True,
    )

    # --- LEFT (-X) face: three button holes (THROUGH) -----------------------
    p_in = comp.constructionPlanes.createInput()
    p_in.setByOffset(comp.yZConstructionPlane, vi(-ext_w / 2))
    p_left = comp.constructionPlanes.add(p_in)
    for z_mm in CONFIG['left_btn_z']:
        cut_circle_on_plane(
            comp, p_left,
            cx=mm(CONFIG['left_btn_y']) - ext_d / 2,
            cy=mm(z_mm),
            dia=mm(CONFIG['left_btn_d']),
            depth=wall_t, through=True,
        )


def add_corner_bosses(comp, ext_w, ext_d):
    """Four M3 bosses standing on the cavity floor at the interior top corners."""
    wall_t = mm(CONFIG['wall_t'])
    cav_w  = mm(CONFIG['cavity_w'])
    cav_d  = mm(CONFIG['cavity_d'])
    boss_h = mm(CONFIG['boss_h'])
    od     = mm(CONFIG['boss_od'])
    iid    = mm(CONFIG['boss_id'])

    # Plane at cavity floor — Z = wall_t (interior bottom)
    p_in = comp.constructionPlanes.createInput()
    p_in.setByOffset(comp.xYConstructionPlane, vi(wall_t))
    p_floor = comp.constructionPlanes.add(p_in)

    inset = od / 2 + 0.05
    half_w = cav_w / 2 - inset
    half_d = cav_d / 2 - inset

    for sx in (-1, 1):
        for sy in (-1, 1):
            cx = sx * half_w
            cy = sy * half_d
            # Solid cylinder — extrude up by boss_h. Try positive distance
            # first; if the floor plane's normal is the other way, retry with
            # negative so the boss grows INTO the cavity rather than down
            # into the bottom wall.
            sk = comp.sketches.add(p_floor)
            sk.sketchCurves.sketchCircles.addByCenterRadius(pt(cx, cy, 0), od / 2)
            prof = sk.profiles.item(0)
            for sign in (1, -1):
                try:
                    ein = comp.features.extrudeFeatures.createInput(
                        prof, adsk.fusion.FeatureOperations.JoinFeatureOperation)
                    ein.setDistanceExtent(False, vi(sign * boss_h))
                    comp.features.extrudeFeatures.add(ein)
                    break
                except Exception:
                    continue
            # Tap-drill hole up through the boss (cut direction unknown a priori
            # — the floor construction plane's normal might point either way).
            sk2 = comp.sketches.add(p_floor)
            sk2.sketchCurves.sketchCircles.addByCenterRadius(pt(cx, cy, 0), iid / 2)
            prof2 = sk2.profiles.item(0)
            for sign in (1, -1):
                try:
                    cin = comp.features.extrudeFeatures.createInput(
                        prof2, adsk.fusion.FeatureOperations.CutFeatureOperation)
                    cin.setDistanceExtent(False, vi(sign * boss_h))
                    comp.features.extrudeFeatures.add(cin)
                    break
                except Exception:
                    continue


def make_lid(parent_comp, ext_w, ext_d, lid_t, box_top_z):
    """Removable lid — body in the root component.

    Originally a sub-component; flattened to bodies-only to work in Fusion 360
    Part Design mode (which only allows one component per document).
    """
    lid_comp = parent_comp

    # The lid plate sits on top of the box (its bottom at box_top_z).
    p_in = lid_comp.constructionPlanes.createInput()
    p_in.setByOffset(lid_comp.xYConstructionPlane, vi(box_top_z))
    p_bot = lid_comp.constructionPlanes.add(p_in)

    sk = lid_comp.sketches.add(p_bot)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        pt(-ext_w / 2, -ext_d / 2, 0),
        pt( ext_w / 2,  ext_d / 2, 0))
    prof = sk.profiles.item(0)
    ein = lid_comp.features.extrudeFeatures.createInput(
        prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ein.setDistanceExtent(False, vi(lid_t))
    feat = lid_comp.features.extrudeFeatures.add(ein)
    lid_body = feat.bodies.item(0)
    lid_body.name = 'lid'

    # ---- Centred 14mm cable hole (THROUGH the lid) ------------------------
    p_top_in = lid_comp.constructionPlanes.createInput()
    p_top_in.setByOffset(lid_comp.xYConstructionPlane, vi(box_top_z + lid_t))
    p_top = lid_comp.constructionPlanes.add(p_top_in)
    cut_circle_on_plane(
        lid_comp, p_top,
        cx=0, cy=0,
        dia=mm(CONFIG['lidar_cable_hole_d']),
        depth=lid_t, through=True,
    )

    # ---- 40mm LIDAR seat recess on TOP face (partial — try both dirs) -----
    cut_circle_on_plane(
        lid_comp, p_top,
        cx=0, cy=0,
        dia=mm(CONFIG['lidar_seat_d']),
        depth=mm(CONFIG['lidar_seat_depth']),
    )

    # ---- Four M3 clearance through-holes at corners (THROUGH) -------------
    inset = mm(CONFIG['lid_screw_inset'])
    half_w = ext_w / 2 - inset
    half_d = ext_d / 2 - inset
    for sx in (-1, 1):
        for sy in (-1, 1):
            cut_circle_on_plane(
                lid_comp, p_top,
                cx=sx * half_w, cy=sy * half_d,
                dia=mm(CONFIG['lid_screw_d']),
                depth=lid_t, through=True,
            )

    return lid_comp, lid_body


def make_grip(parent_comp, ext_d, wall_t):
    """Grip swept along a 15°-tilted path with ParallelOrientationType.

    Why a sweep instead of extrude-then-rotate: rotating an already-extruded
    grip about an X-axis pivot at its top tilts the WHOLE top face, including
    the -Y edge which then dips below z=0 by ~gd*sin(15°) ≈ 9mm. The result is
    the top face no longer sits flush against the box bottom.

    Using a sweep with ParallelOrientationType keeps the cross-section parallel
    to the XY plane throughout the entire sweep, so the top face stays at z=0
    (flush with the box bottom) and only the LOCATION of the body migrates
    along the tilted path.

    Path: a straight line on the YZ plane from (0, grip_cy, 0) tilted 15°
    toward -Y, length grip_len.
    Cross-section: rounded-rect on the XY plane, centred at (0, grip_cy).
    """
    grip_comp = parent_comp

    gw = mm(CONFIG['grip_cross_w'])
    gd = mm(CONFIG['grip_cross_d'])
    gr = mm(CONFIG['grip_corner_r'])
    gl = mm(CONFIG['grip_len'])
    angle = math.radians(CONFIG['grip_angle_deg'])

    grip_cx = 0.0
    grip_cy = -ext_d / 2 + gd / 2

    # --- 1. Cross-section sketch on XY plane (centred at (grip_cx, grip_cy)) ---
    sk = grip_comp.sketches.add(grip_comp.xYConstructionPlane)
    lx, ly = grip_cx - gw / 2, grip_cy - gd / 2
    rx, ry = grip_cx + gw / 2, grip_cy + gd / 2
    lines = sk.sketchCurves.sketchLines
    arcs  = sk.sketchCurves.sketchArcs
    lines.addByTwoPoints(pt(lx + gr, ly, 0), pt(rx - gr, ly, 0))    # bottom (-Y)
    lines.addByTwoPoints(pt(rx, ly + gr, 0), pt(rx, ry - gr, 0))    # right  (+X)
    lines.addByTwoPoints(pt(rx - gr, ry, 0), pt(lx + gr, ry, 0))    # top    (+Y)
    lines.addByTwoPoints(pt(lx, ry - gr, 0), pt(lx, ly + gr, 0))    # left   (-X)
    h = gr * math.cos(math.radians(45))
    arcs.addByThreePoints(pt(rx - gr, ly, 0),
                          pt(rx - gr + h, ly + gr - h, 0),
                          pt(rx, ly + gr, 0))
    arcs.addByThreePoints(pt(rx, ry - gr, 0),
                          pt(rx - gr + h, ry - gr + h, 0),
                          pt(rx - gr, ry, 0))
    arcs.addByThreePoints(pt(lx + gr, ry, 0),
                          pt(lx + gr - h, ry - gr + h, 0),
                          pt(lx, ry - gr, 0))
    arcs.addByThreePoints(pt(lx, ly + gr, 0),
                          pt(lx + gr - h, ly + gr - h, 0),
                          pt(lx + gr, ly, 0))

    prof = None
    for p in sk.profiles:
        if prof is None or p.areaProperties().area > prof.areaProperties().area:
            prof = p
    if prof is None:
        raise RuntimeError('Could not form grip cross-section profile')

    # --- 2. Path sketch on YZ plane — straight line tilted 15° toward -Y ---
    # Start at the cross-section's centre, end gl away tilted by `angle`.
    path_sk = grip_comp.sketches.add(grip_comp.yZConstructionPlane)
    # YZ sketch's local axes: u = world Y, v = world Z
    p_start = pt(grip_cy, 0.0, 0.0)
    p_end   = pt(grip_cy - gl * math.sin(angle),
                 -gl * math.cos(angle),
                 0.0)
    path_line = path_sk.sketchCurves.sketchLines.addByTwoPoints(p_start, p_end)
    try:
        path = grip_comp.features.createPath(path_line)
    except Exception:
        return grip_comp, None

    # --- 3. Sweep with ParallelOrientationType ---
    # ParallelOrientationType keeps the profile orientation constant (parallel
    # to its sketch plane / XY) along the whole path. The body's top face
    # therefore remains coplanar with z=0 and flush with the box bottom.
    sw_in = grip_comp.features.sweepFeatures.createInput(
        prof, path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    try:
        sw_in.orientation = adsk.fusion.SweepOrientationTypes.ParallelOrientationType
    except Exception:
        pass
    try:
        feat = grip_comp.features.sweepFeatures.add(sw_in)
        grip_body = feat.bodies.item(0)
        grip_body.name = 'grip_body'
    except Exception:
        # Fallback to straight extrude if sweep rejects the inputs.
        ein = grip_comp.features.extrudeFeatures.createInput(
            prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ein.setDistanceExtent(False, vi(-gl))
        feat = grip_comp.features.extrudeFeatures.add(ein)
        grip_body = feat.bodies.item(0)
        grip_body.name = 'grip_body'

    # --- 4. Hollow the grip — shell from the topmost +Z face (z ≈ 0) ---
    try:
        top_grip_face = None
        for f in grip_body.faces:
            if isinstance(f.geometry, adsk.core.Plane):
                n = f.geometry.normal
                if n.z > 0.85:
                    if top_grip_face is None or f.geometry.origin.z > top_grip_face.geometry.origin.z:
                        top_grip_face = f
        if top_grip_face is not None:
            shin = grip_comp.features.shellFeatures.createInput(oc([top_grip_face]), False)
            shin.insideThickness = vi(mm(CONFIG['grip_wall_t']))
            grip_comp.features.shellFeatures.add(shin)
    except Exception:
        pass

    # --- 5. Round the bottom edges (best-effort) ---
    try:
        bot_z_threshold = -gl * math.cos(angle) * 0.9
        bot_edges = oc([])
        for e in grip_body.edges:
            sp = e.startVertex.geometry
            ep = e.endVertex.geometry
            if sp.z < bot_z_threshold and ep.z < bot_z_threshold:
                bot_edges.add(e)
        if bot_edges.count > 0:
            fin = grip_comp.features.filletFeatures.createInput()
            fin.addConstantRadiusEdgeSet(bot_edges, vi(mm(6.0)), True)
            grip_comp.features.filletFeatures.add(fin)
    except Exception:
        pass

    return grip_comp, grip_body


def make_trigger_guard(parent_comp, ext_d, wall_t):
    """Trigger guard — sweep a thin tube along an arc in the YZ plane.

    The guard is anchored at the GRIP's front-top edge (at y = -ext_d/2 + gd,
    z = 0) — not the BOX's front-bottom corner. The original anchor at the
    box front put the guard 165mm away from the grip, which looks nothing
    like a pistol. A real trigger guard wraps just in front of the grip.

    Arc start:  (X=0, Y=anchor_y,       Z=0)        — grip's front-top edge
    Arc mid:    (X=0, Y=anchor_y + fwd, Z=-down/2)
    Arc end:    (X=0, Y=anchor_y,       Z=-down)
    """
    tg_comp = parent_comp

    fwd  = mm(CONFIG['trigger_fwd'])
    down = mm(CONFIG['trigger_down'])
    tube = mm(CONFIG['trigger_tube_d'])
    gd   = mm(CONFIG['grip_cross_d'])

    # Grip's front-top edge: grip_cy = -ext_d/2 + gd/2, front face is +gd/2 away.
    anchor_y = -ext_d / 2 + gd

    # --- Path sketch on YZ plane ---
    path_sk = tg_comp.sketches.add(tg_comp.yZConstructionPlane)
    # YZ sketch axes: u = world Y, v = world Z
    arc = path_sk.sketchCurves.sketchArcs.addByThreePoints(
        pt(anchor_y,       0,         0),
        pt(anchor_y + fwd, -down / 2, 0),
        pt(anchor_y,       -down,     0))
    try:
        path = tg_comp.features.createPath(arc)
    except Exception:
        return tg_comp

    # --- Profile sketch: thin circle on plane perpendicular to the arc start ---
    # Plane = parallel to xZ (the plane normal to Y) offset to anchor_y.
    p_in = tg_comp.constructionPlanes.createInput()
    p_in.setByOffset(tg_comp.xZConstructionPlane, vi(anchor_y))
    p_perp = tg_comp.constructionPlanes.add(p_in)
    sk_prof = tg_comp.sketches.add(p_perp)
    sk_prof.sketchCurves.sketchCircles.addByCenterRadius(pt(0, 0, 0), tube / 2)
    circle_prof = sk_prof.profiles.item(0)

    sw_in = tg_comp.features.sweepFeatures.createInput(
        circle_prof, path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    try:
        sw_in.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
    except Exception:
        pass
    try:
        feat = tg_comp.features.sweepFeatures.add(sw_in)
        feat.bodies.item(0).name = 'trigger_guard'
    except Exception:
        pass

    return tg_comp


def add_knurling(parent_comp):
    """Decorative bumps on the grip front face. Best-effort — failure here is
    non-fatal so the rest of the script still runs. Bodies live in root."""
    grip_comp = parent_comp

    try:
        cav_d  = mm(CONFIG['cavity_d'])
        wall_t = mm(CONFIG['wall_t'])
        grip_cy = -(cav_d + 2 * wall_t) / 2 + mm(CONFIG['grip_cross_d']) / 2
        bump_d = mm(CONFIG['knurl_bump_d'])
        bump_h = mm(CONFIG['knurl_bump_h'])
        pitch  = mm(CONFIG['knurl_pitch'])
        cols   = CONFIG['knurl_cols']
        rows   = CONFIG['knurl_rows']

        p_in = grip_comp.constructionPlanes.createInput()
        p_in.setByOffset(grip_comp.xZConstructionPlane,
                          vi(grip_cy + mm(CONFIG['grip_cross_d']) / 2))
        plane = grip_comp.constructionPlanes.add(p_in)

        sk = grip_comp.sketches.add(plane)
        sk.sketchCurves.sketchCircles.addByCenterRadius(pt(0, -mm(60), 0), bump_d / 2)
        prof = sk.profiles.item(0)
        ein = grip_comp.features.extrudeFeatures.createInput(
            prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ein.setDistanceExtent(False, vi(bump_h))
        bump_feat = grip_comp.features.extrudeFeatures.add(ein)

        rp_in = grip_comp.features.rectangularPatternFeatures.createInput(
            oc([bump_feat]),
            grip_comp.xConstructionAxis,
            vi(cols), vi(pitch),
            adsk.fusion.PatternDistanceType.SpacingPatternDistanceType)
        rp_in.setDirectionTwo(grip_comp.zConstructionAxis, vi(rows), vi(pitch))
        grip_comp.features.rectangularPatternFeatures.add(rp_in)
    except Exception:
        pass


def make_reference_bodies(parent_comp, ext_w, ext_d, wall_t):
    """Translucent reference bodies for battery / RPi5 / ESP32.

    Body names are prefixed `ref_…` so they're easy to filter / hide in the
    browser. (Flattened from a sub-component to root-level bodies for Part
    Design mode compatibility.)
    """
    ref = parent_comp

    cav_w = mm(CONFIG['cavity_w'])
    cav_d = mm(CONFIG['cavity_d'])

    cav_floor_z = wall_t

    # ----- Battery: centred in XY, long axis = +Y -----
    bw = mm(CONFIG['batt_w'])   # along Y
    bd = mm(CONFIG['batt_d'])   # along X
    bh = mm(CONFIG['batt_h'])
    _make_named_box(ref, 'ref_battery_red',
                    x0=-bd / 2, y0=-bw / 2, z0=cav_floor_z,
                    w=bd, d=bw, h=bh)

    # ----- Raspberry Pi 5: centred in X, shifted toward +Y -----
    py = mm(CONFIG['rpi_y'])
    px = mm(CONFIG['rpi_x'])
    pz = mm(CONFIG['rpi_z'])
    rpi_z0 = wall_t + mm(31.0)         # Layer 2 starts 31 mm above cavity floor
    rpi_y0 = cav_d / 2 - py - mm(5.0)  # 5 mm gap from +Y face
    _make_named_box(ref, 'ref_rpi5_green',
                    x0=-px / 2, y0=rpi_y0, z0=rpi_z0,
                    w=px, d=py, h=pz)

    # ----- ESP32 DevKit: at (X_mid - 14mm, Y=10mm-from-front, Z=65mm) -----
    ey = mm(CONFIG['esp_y'])
    ex = mm(CONFIG['esp_x'])
    ez = mm(CONFIG['esp_z'])
    esp_x0 = -mm(14.0) - ex / 2
    esp_y0 = cav_d / 2 - mm(10.0) - ey
    esp_z0 = wall_t + mm(65.0)
    _make_named_box(ref, 'ref_esp32_blue',
                    x0=esp_x0, y0=esp_y0, z0=esp_z0,
                    w=ex, d=ey, h=ez)

    return ref


def _make_named_box(comp, name, x0, y0, z0, w, d, h):
    """Sketch + extrude a named rectangular box body, base at z0."""
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        pt(x0, y0, 0), pt(x0 + w, y0 + d, 0))
    prof = sk.profiles.item(0)
    ein = comp.features.extrudeFeatures.createInput(
        prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ein.startExtent = adsk.fusion.OffsetStartDefinition.create(vi(z0))
    ein.setDistanceExtent(False, vi(h))
    feat = comp.features.extrudeFeatures.add(ein)
    feat.bodies.item(0).name = name
    return feat.bodies.item(0)


def apply_appearances(app, design, root_enclosure):
    """Best-effort coloured / translucent appearances. Skipped quietly if the
    Fusion appearance library lookups fail (varies by Fusion version)."""
    try:
        fusion_libs = app.materialLibraries.itemByName('Fusion 360 Appearance Library')
        if fusion_libs is None:
            return
        appearances = fusion_libs.appearances

        def get_app(name):
            try:
                return appearances.itemByName(name)
            except Exception:
                return None

        matte_black = get_app('Plastic - Matte (Black)')
        dark_grey   = get_app('Plastic - Matte (Dark Grey)') or get_app('Plastic - Matte (Grey)')
        red_tr      = get_app('Plastic - Translucent Glossy (Red)')
        green_tr    = get_app('Plastic - Translucent Glossy (Green)')
        blue_tr     = get_app('Plastic - Translucent Glossy (Blue)')

        def apply_to_body_name(comp, body_name, appearance):
            if appearance is None:
                return
            for b in comp.bRepBodies:
                if b.name == body_name:
                    try:
                        b.appearance = appearance
                    except Exception:
                        pass
            for o in comp.occurrences:
                apply_to_body_name(o.component, body_name, appearance)

        apply_to_body_name(root_enclosure, 'enclosure_body',  matte_black)
        apply_to_body_name(root_enclosure, 'lid',             dark_grey)
        apply_to_body_name(root_enclosure, 'grip_body',       matte_black)
        apply_to_body_name(root_enclosure, 'trigger_guard',   matte_black)
        apply_to_body_name(root_enclosure, 'ref_battery_red', red_tr)
        apply_to_body_name(root_enclosure, 'ref_rpi5_green',  green_tr)
        apply_to_body_name(root_enclosure, 'ref_esp32_blue',  blue_tr)
    except Exception:
        pass


def fillet_exterior(comp, body):
    """Best-effort 2 mm fillet on every vertical exterior edge."""
    try:
        edges = oc([])
        for e in body.edges:
            sp = e.startVertex.geometry
            ep = e.endVertex.geometry
            dx = ep.x - sp.x
            dy = ep.y - sp.y
            dz = ep.z - sp.z
            length = math.sqrt(dx * dx + dy * dy + dz * dz)
            if length < 1e-3:
                continue
            if abs(dz) / length > 0.9:
                edges.add(e)
        if edges.count == 0:
            return
        fin = comp.features.filletFeatures.createInput()
        fin.addConstantRadiusEdgeSet(edges, vi(mm(CONFIG['fillet_exterior'])), True)
        comp.features.filletFeatures.add(fin)
    except Exception:
        pass


# =============================================================================
# Entry point
# =============================================================================

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        if not design:
            if ui:
                ui.messageBox('No active Fusion 360 design — start a new design first.')
            return

        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        timeline = design.timeline
        root = design.rootComponent

        # ------------------------------------------------------------------
        # All bodies live in the ROOT component. Originally each major piece
        # (Lid / Grip / TriggerGuard / reference_only) was its own
        # sub-component, but Fusion 360 Part Design mode rejects
        # `root.occurrences.addNewComponent(...)` with:
        #   "Part Design documents can only contain one component, please
        #    add this Part to an Assembly to add multiple components."
        # Body names are descriptive (`enclosure_body`, `lid`, `grip_body`,
        # `trigger_guard`, `ref_battery_red`, …) so the browser tree stays
        # readable. Switch the document type to Assembly Design if you want
        # the original component hierarchy.
        # ------------------------------------------------------------------
        if root.name != 'ReconScannerEnclosure':
            try:
                root.name = 'ReconScannerEnclosure'
            except Exception:
                pass   # rename can fail in some doc types; harmless

        # 1. Body box — outer extrude + shell off top face
        t0 = timeline.markerPosition
        body, ext_w, ext_d, ext_h = make_body_box(root)
        group_timeline(timeline, t0, 'BodyBoxShell')

        # 2. Face cuts
        t0 = timeline.markerPosition
        cut_face_features(root, ext_w, ext_d, ext_h)
        group_timeline(timeline, t0, 'FaceCuts')

        # 3. Interior corner bosses for M3 lid screws
        t0 = timeline.markerPosition
        add_corner_bosses(root, ext_w, ext_d)
        group_timeline(timeline, t0, 'CornerBosses')

        # 4. Removable lid (separate body)
        t0 = timeline.markerPosition
        make_lid(root, ext_w, ext_d, mm(CONFIG['lid_thickness']), ext_h)
        group_timeline(timeline, t0, 'Lid')

        # 5. Grip (separate body)
        t0 = timeline.markerPosition
        make_grip(root, ext_d, mm(CONFIG['wall_t']))
        group_timeline(timeline, t0, 'Grip')

        # 6. Trigger guard (separate body)
        t0 = timeline.markerPosition
        make_trigger_guard(root, ext_d, mm(CONFIG['wall_t']))
        group_timeline(timeline, t0, 'TriggerGuard')

        # 7. Knurling bumps on grip (decorative; best-effort)
        t0 = timeline.markerPosition
        add_knurling(root)
        group_timeline(timeline, t0, 'Knurling')

        # 8. Reference bodies — translucent battery / RPi5 / ESP32
        t0 = timeline.markerPosition
        make_reference_bodies(root, ext_w, ext_d, mm(CONFIG['wall_t']))
        group_timeline(timeline, t0, 'ReferenceBodies')

        # 9. Appearances (best-effort)
        apply_appearances(app, design, root)

        # 10. Exterior 2 mm fillets (printability)
        t0 = timeline.markerPosition
        fillet_exterior(root, body)
        group_timeline(timeline, t0, 'ExteriorFillets')

        if ui:
            ui.messageBox(
                'Recon Scanner Enclosure generated.\n\n'
                'Bodies in the root component (Part Design mode):\n'
                '  enclosure_body    — main shelled box\n'
                '  lid               — removable top plate\n'
                '  grip_body         — pistol grip\n'
                '  trigger_guard     — swept tube\n'
                '  ref_battery_red   — translucent reference\n'
                '  ref_rpi5_green    — translucent reference\n'
                '  ref_esp32_blue    — translucent reference\n\n'
                'For sub-component hierarchy, create a new Assembly Design\n'
                'document and re-run the script there.')

    except:
        if ui:
            ui.messageBox('Error: ' + traceback.format_exc())


def stop(context):
    pass
