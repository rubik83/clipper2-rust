//! Engine free functions - standalone utility functions for the sweep-line algorithm
//!
//! Direct port from clipper.engine.cpp
//! Copyright (c) Angus Johnson 2010-2025

use crate::core::*;
use crate::engine::*;

// ============================================================================
// Engine Free Functions - Direct port from clipper.engine.cpp
// ============================================================================

/// Check if a value is odd
/// Direct port from clipper.engine.cpp line 61
#[inline]
pub fn is_odd(val: i32) -> bool {
    val & 1 != 0
}

/// Check if an active edge is "hot" (has an output record)
/// Direct port from clipper.engine.cpp line 67
#[inline]
pub fn is_hot_edge(e: &Active) -> bool {
    e.outrec != NONE
}

/// Check if an active edge is part of an open path
/// Direct port from clipper.engine.cpp line 73
#[inline]
pub fn is_open_active(e: &Active, minima_list: &[LocalMinima]) -> bool {
    idx(minima_list, e.local_min).is_open
}

/// Check if a vertex is an open end
/// Direct port from clipper.engine.cpp line 79
#[inline]
pub fn is_open_end_vertex(v: &Vertex) -> bool {
    (v.flags & (VertexFlags::OPEN_START | VertexFlags::OPEN_END)) != VertexFlags::EMPTY
}

/// Check if an active edge's vertex_top is an open end
/// Direct port from clipper.engine.cpp line 87
#[inline]
pub fn is_open_end_active(e: &Active, vertex_arena: &[Vertex]) -> bool {
    is_open_end_vertex(idx(vertex_arena, e.vertex_top))
}

/// Calculate slope dx for an edge defined by two points
/// Direct port from clipper.engine.cpp line 116
#[inline]
pub fn get_dx(pt1: Point64, pt2: Point64) -> f64 {
    let dy = (pt2.y - pt1.y) as f64;
    if dy != 0.0 {
        (pt2.x - pt1.x) as f64 / dy
    } else if pt2.x > pt1.x {
        -f64::MAX
    } else {
        f64::MAX
    }
}

/// Get the x coordinate of an active edge at a given y
/// Direct port from clipper.engine.cpp line 127
#[inline]
pub fn top_x(ae: &Active, current_y: i64) -> i64 {
    if current_y == ae.top.y || ae.top.x == ae.bot.x {
        ae.top.x
    } else if current_y == ae.bot.y {
        ae.bot.x
    } else {
        ae.bot.x + crate::core::nearbyint_f64(ae.dx * (current_y - ae.bot.y) as f64) as i64
    }
}

/// Check if an active edge is horizontal
/// Direct port from clipper.engine.cpp line 137
#[inline]
pub fn is_horizontal_active(e: &Active) -> bool {
    e.top.y == e.bot.y
}

/// Check if a horizontal edge is heading right
/// Direct port from clipper.engine.cpp line 143
#[inline]
pub fn is_heading_right_horz(e: &Active) -> bool {
    e.dx == -f64::MAX
}

/// Check if a horizontal edge is heading left
/// Direct port from clipper.engine.cpp line 149
#[inline]
pub fn is_heading_left_horz(e: &Active) -> bool {
    e.dx == f64::MAX
}

/// Get the polytype of an active edge
/// Direct port from clipper.engine.cpp line 162
#[inline]
pub fn get_poly_type(e: &Active, minima_list: &[LocalMinima]) -> PathType {
    idx(minima_list, e.local_min).polytype
}

/// Check if two active edges are the same polytype
/// Direct port from clipper.engine.cpp line 167
#[inline]
pub fn is_same_poly_type(e1: &Active, e2: &Active, minima_list: &[LocalMinima]) -> bool {
    idx(minima_list, e1.local_min).polytype == idx(minima_list, e2.local_min).polytype
}

/// Set dx for an active edge from its bot/top points
/// Direct port from clipper.engine.cpp line 172
#[inline]
pub fn set_dx(e: &mut Active) {
    e.dx = get_dx(e.bot, e.top);
}

/// Get next vertex for an active edge (depends on winding direction)
/// Direct port from clipper.engine.cpp line 177
#[inline]
pub fn next_vertex(e: &Active, vertex_arena: &[Vertex]) -> Idx {
    if e.wind_dx > 0 {
        idx(vertex_arena, e.vertex_top).next
    } else {
        idx(vertex_arena, e.vertex_top).prev
    }
}

/// Get the vertex two steps back (useful for alternate edge top)
/// Direct port from clipper.engine.cpp line 187
#[inline]
pub fn prev_prev_vertex(ae: &Active, vertex_arena: &[Vertex]) -> Idx {
    if ae.wind_dx > 0 {
        let prev = idx(vertex_arena, ae.vertex_top).prev;
        idx(vertex_arena, prev).prev
    } else {
        let next = idx(vertex_arena, ae.vertex_top).next;
        idx(vertex_arena, next).next
    }
}

/// Check if a vertex is a local maximum
/// Direct port from clipper.engine.cpp line 215
#[inline]
pub fn is_maxima_vertex(v: &Vertex) -> bool {
    (v.flags & VertexFlags::LOCAL_MAX) != VertexFlags::EMPTY
}

/// Check if an active edge is at a local maximum
/// Direct port from clipper.engine.cpp line 221
#[inline]
pub fn is_maxima_active(e: &Active, vertex_arena: &[Vertex]) -> bool {
    is_maxima_vertex(idx(vertex_arena, e.vertex_top))
}

/// Count points in an OutPt circular list
/// Direct port from clipper.engine.cpp line 266
pub fn point_count(op_start: Idx, outpt_arena: &[OutPt]) -> i32 {
    let mut op2 = op_start;
    let mut cnt = 0;
    loop {
        op2 = idx(outpt_arena, op2).next;
        cnt += 1;
        if op2 == op_start {
            break;
        }
    }
    cnt
}

/// Check if an OutPt path is invalid (single point or None)
/// Direct port from clipper.engine.cpp line 105
#[inline]
pub fn is_invalid_path(op: Option<Idx>, outpt_arena: &[OutPt]) -> bool {
    match op {
        None => true,
        Some(cur_idx) => idx(outpt_arena, cur_idx).next == cur_idx,
    }
}

/// Calculate area of an OutPt circular list
/// Direct port from clipper.engine.cpp line 366
pub fn area_outpt(op_start: Idx, outpt_arena: &[OutPt]) -> f64 {
    let mut result = 0.0;
    let mut op2 = op_start;
    loop {
        let prev_idx = idx(outpt_arena, op2).prev;
        result += (idx(outpt_arena, prev_idx).pt.y + idx(outpt_arena, op2).pt.y) as f64
            * (idx(outpt_arena, prev_idx).pt.x - idx(outpt_arena, op2).pt.x) as f64;
        op2 = idx(outpt_arena, op2).next;
        if op2 == op_start {
            break;
        }
    }
    result * 0.5
}

/// Calculate area of a triangle formed by three points
/// Direct port from clipper.engine.cpp line 380
#[inline]
pub fn area_triangle(pt1: Point64, pt2: Point64, pt3: Point64) -> f64 {
    (pt3.y + pt1.y) as f64 * (pt3.x - pt1.x) as f64
        + (pt1.y + pt2.y) as f64 * (pt1.x - pt2.x) as f64
        + (pt2.y + pt3.y) as f64 * (pt2.x - pt3.x) as f64
}

/// Reverse the direction of an OutPt circular list
/// Direct port from clipper.engine.cpp line 388
pub fn reverse_out_pts(op_start: Idx, outpt_arena: &mut [OutPt]) {
    let mut op1 = op_start;
    loop {
        {
            let op = idx_mut(outpt_arena, op1);
            std::mem::swap(&mut op.next, &mut op.prev);
        }
        op1 = idx(outpt_arena, op1).prev; // advance to original next (now in prev)
        if op1 == op_start {
            break;
        }
    }
}

/// Sort comparator for IntersectNodes
/// Direct port from clipper.engine.cpp line 322
pub fn intersect_list_sort(a: &IntersectNode, b: &IntersectNode) -> std::cmp::Ordering {
    if a.pt.y == b.pt.y {
        a.pt.x.cmp(&b.pt.x)
    } else {
        b.pt.y.cmp(&a.pt.y)
    }
}

/// Get the maxima pair for an active edge
/// Direct port from clipper.engine.cpp line 254
pub fn get_maxima_pair(e: &Active, active_arena: &[Active]) -> Idx {
    let mut cur_idx = e.next_in_ael;
    while cur_idx != NONE {
        if idx(active_arena, cur_idx).vertex_top == e.vertex_top {
            return cur_idx;
        }
        cur_idx = idx(active_arena, cur_idx).next_in_ael;
    }
    NONE
}

/// Get current Y maxima vertex for open paths
/// Direct port from clipper.engine.cpp line 226
pub fn get_curr_y_maxima_vertex_open(e: &Active, vertex_arena: &[Vertex]) -> Option<Idx> {
    let mut result = e.vertex_top;
    if e.wind_dx > 0 {
        while idx(vertex_arena, idx(vertex_arena, result).next).pt.y
            == idx(vertex_arena, result).pt.y
            && (idx(vertex_arena, result).flags & (VertexFlags::OPEN_END | VertexFlags::LOCAL_MAX))
                == VertexFlags::EMPTY
        {
            result = idx(vertex_arena, result).next;
        }
    } else {
        while idx(vertex_arena, idx(vertex_arena, result).prev).pt.y
            == idx(vertex_arena, result).pt.y
            && (idx(vertex_arena, result).flags & (VertexFlags::OPEN_END | VertexFlags::LOCAL_MAX))
                == VertexFlags::EMPTY
        {
            result = idx(vertex_arena, result).prev;
        }
    }
    if is_maxima_vertex(idx(vertex_arena, result)) {
        Some(result)
    } else {
        None
    }
}

/// Get current Y maxima vertex for closed paths
/// Direct port from clipper.engine.cpp line 243
pub fn get_curr_y_maxima_vertex(e: &Active, vertex_arena: &[Vertex]) -> Option<Idx> {
    let mut result = e.vertex_top;
    if e.wind_dx > 0 {
        while idx(vertex_arena, idx(vertex_arena, result).next).pt.y
            == idx(vertex_arena, result).pt.y
        {
            result = idx(vertex_arena, result).next;
        }
    } else {
        while idx(vertex_arena, idx(vertex_arena, result).prev).pt.y
            == idx(vertex_arena, result).pt.y
        {
            result = idx(vertex_arena, result).prev;
        }
    }
    if is_maxima_vertex(idx(vertex_arena, result)) {
        Some(result)
    } else {
        None
    }
}

/// Check if the AEL ordering is valid between two active edges
/// Direct port from clipper.engine.cpp IsValidAelOrder (line 1119)
pub fn is_valid_ael_order(
    resident: &Active,
    newcomer: &Active,
    vertex_arena: &[Vertex],
    minima_list: &[LocalMinima],
) -> bool {
    if newcomer.curr_x != resident.curr_x {
        return newcomer.curr_x > resident.curr_x;
    }

    // get the turning direction: resident.top, newcomer.bot, newcomer.top
    let i = cross_product_sign(
        idx(vertex_arena, resident.vertex_top).pt,
        newcomer.bot,
        idx(vertex_arena, newcomer.vertex_top).pt,
    );
    if i != 0 {
        return i < 0;
    }

    // edges must be collinear to get here
    // for starting open paths, place them according to
    // the direction they're about to turn
    if !is_maxima_active(resident, vertex_arena)
        && idx(vertex_arena, resident.vertex_top).pt.y > idx(vertex_arena, newcomer.vertex_top).pt.y
    {
        let nv = next_vertex(resident, vertex_arena);
        return cross_product_sign(
            newcomer.bot,
            idx(vertex_arena, resident.vertex_top).pt,
            idx(vertex_arena, nv).pt,
        ) <= 0;
    } else if !is_maxima_active(newcomer, vertex_arena)
        && idx(vertex_arena, newcomer.vertex_top).pt.y > idx(vertex_arena, resident.vertex_top).pt.y
    {
        let nv = next_vertex(newcomer, vertex_arena);
        return cross_product_sign(
            newcomer.bot,
            idx(vertex_arena, newcomer.vertex_top).pt,
            idx(vertex_arena, nv).pt,
        ) >= 0;
    }

    let y = newcomer.bot.y;
    let newcomer_is_left = newcomer.is_left_bound;

    if resident.bot.y != y
        || idx(vertex_arena, idx(minima_list, resident.local_min).vertex)
            .pt
            .y
            != y
    {
        newcomer.is_left_bound
    } else if resident.is_left_bound != newcomer_is_left {
        newcomer_is_left
    } else if is_collinear(
        idx(vertex_arena, prev_prev_vertex(resident, vertex_arena)).pt,
        resident.bot,
        idx(vertex_arena, resident.vertex_top).pt,
    ) {
        true
    } else {
        // compare turning direction of the alternate bound
        let ppv_r = prev_prev_vertex(resident, vertex_arena);
        let ppv_n = prev_prev_vertex(newcomer, vertex_arena);
        (cross_product_sign(
            idx(vertex_arena, ppv_r).pt,
            newcomer.bot,
            idx(vertex_arena, ppv_n).pt,
        ) > 0)
            == newcomer_is_left
    }
}

// ============================================================================
// Path building functions
// ============================================================================

/// Build a Path64 from an OutRec
/// Direct port from clipper.engine.cpp GetCleanPath equivalent
pub fn get_clean_path(op_start: Idx, outpt_arena: &[OutPt]) -> Path64 {
    let mut result = Path64::new();
    let mut op = op_start;
    loop {
        let prev = idx(outpt_arena, op).prev;
        let next = idx(outpt_arena, op).next;
        if idx(outpt_arena, op).pt == idx(outpt_arena, prev).pt
            || is_collinear(
                idx(outpt_arena, prev).pt,
                idx(outpt_arena, op).pt,
                idx(outpt_arena, next).pt,
            )
        {
            op = next;
            if op == op_start {
                break;
            }
            continue;
        }
        result.push(idx(outpt_arena, op).pt);
        op = next;
        if op == op_start {
            break;
        }
    }
    result
}

// ============================================================================
// Free helper functions for sweep-line algorithm
// Direct port from clipper.engine.cpp
// ============================================================================

/// Get the real (non-disposed) OutRec by following owner chain
/// Direct port from clipper.engine.cpp GetRealOutRec (line 412)
pub fn get_real_outrec(outrec_list: &[OutRec], outrec_idx: Idx) -> Idx {
    let mut cur = outrec_idx;
    while cur != NONE {
        if idx(outrec_list, cur).pts != NONE {
            return cur;
        }
        cur = idx(outrec_list, cur).owner;
    }
    NONE
}

/// Check if an owner is valid (not circular)
/// Direct port from clipper.engine.cpp IsValidOwner (line 418)
pub fn is_valid_owner(outrec_list: &[OutRec], outrec_idx: Idx, test_owner_idx: Idx) -> bool {
    let mut cur_idx = test_owner_idx;
    while cur_idx != NONE {
        if cur_idx == outrec_idx {
            return false;
        }
        cur_idx = idx(outrec_list, cur_idx).owner;
    }
    true
}

/// Check if two points are really close (within 2 units)
/// Direct port from clipper.engine.cpp PtsReallyClose (line 436)
#[inline]
pub fn pts_really_close(pt1: Point64, pt2: Point64) -> bool {
    (pt1.x - pt2.x).abs() < 2 && (pt1.y - pt2.y).abs() < 2
}

/// Check if an OutPt triangle is very small
/// Direct port from clipper.engine.cpp IsVerySmallTriangle (line 441)
pub fn is_very_small_triangle(op_idx: Idx, outpt_arena: &[OutPt]) -> bool {
    let next = idx(outpt_arena, op_idx).next;
    let prev = idx(outpt_arena, op_idx).prev;
    if idx(outpt_arena, next).next != prev {
        return false;
    }
    pts_really_close(idx(outpt_arena, prev).pt, idx(outpt_arena, next).pt)
        || pts_really_close(idx(outpt_arena, op_idx).pt, idx(outpt_arena, next).pt)
        || pts_really_close(idx(outpt_arena, op_idx).pt, idx(outpt_arena, prev).pt)
}

/// Check if a closed path is valid
/// Direct port from clipper.engine.cpp IsValidClosedPath (line 449)
pub fn is_valid_closed_path(cur_idx: Idx, outpt_arena: &[OutPt]) -> bool {
    if cur_idx == NONE {
        return false;
    }
    let next = idx(outpt_arena, cur_idx).next;
    if next == cur_idx {
        return false;
    }
    let prev = idx(outpt_arena, cur_idx).prev;
    if next == prev {
        return false;
    }
    !is_very_small_triangle(cur_idx, outpt_arena)
}

/// Check if the hot edge is ascending (front edge of its outrec)
/// Direct port from clipper.engine.cpp OutrecIsAscending (line 455)
#[inline]
pub fn outrec_is_ascending(
    hot_edge_idx: Idx,
    outrec_list: &[OutRec],
    active_arena: &[Active],
) -> bool {
    let or_idx = idx(active_arena, hot_edge_idx).outrec;
    if or_idx != NONE {
        idx(outrec_list, or_idx).front_edge == hot_edge_idx
    } else {
        false
    }
}

/// Swap front and back sides of an OutRec
/// Direct port from clipper.engine.cpp SwapFrontBackSides (line 460)
pub fn swap_front_back_sides(outrec: &mut OutRec, outpt_arena: &[OutPt]) {
    std::mem::swap(&mut outrec.front_edge, &mut outrec.back_edge);
    outrec.pts = idx(outpt_arena, outrec.pts).next;
}

/// Check if intersect node edges are adjacent in AEL
/// Direct port from clipper.engine.cpp EdgesAdjacentInAEL (line 468)
#[inline]
pub fn edges_adjacent_in_ael(inode: &IntersectNode, active_arena: &[Active]) -> bool {
    idx(active_arena, inode.edge1).next_in_ael == inode.edge2
        || idx(active_arena, inode.edge1).prev_in_ael == inode.edge2
}

/// Check if an edge is joined
/// Direct port from clipper.engine.cpp IsJoined (line 473)
#[inline]
pub fn is_joined(e: &Active) -> bool {
    e.join_with != JoinWith::NoJoin
}

/// Set the owner of an OutRec
/// Direct port from clipper.engine.cpp SetOwner (line 478)
pub fn set_owner(outrec_list: &mut [OutRec], outrec_idx: Idx, new_owner_idx: Idx) {
    // precondition: new_owner_idx is valid
    // Direct port from C++: new_owner->owner = GetRealOutRec(new_owner->owner);
    // When owner is None, GetRealOutRec returns None (C++ handles nullptr correctly)
    let owner = idx(outrec_list, new_owner_idx).owner;
    idx_mut(outrec_list, new_owner_idx).owner = get_real_outrec(outrec_list, owner);
    let mut tmp = new_owner_idx;
    while tmp != NONE {
        if tmp == outrec_idx {
            let outrec_owner = idx(outrec_list, outrec_idx).owner;
            idx_mut(outrec_list, new_owner_idx).owner = outrec_owner;
            break;
        }
        tmp = idx(outrec_list, tmp).owner;
    }
    idx_mut(outrec_list, outrec_idx).owner = new_owner_idx;
}

/// Point in polygon test for OutPt-based polygons
/// Direct port from clipper.engine.cpp PointInOpPolygon (line 488)
pub fn point_in_op_polygon(
    pt: Point64,
    op_start: Idx,
    outpt_arena: &[OutPt],
) -> PointInPolygonResult {
    let next = idx(outpt_arena, op_start).next;
    if next == op_start || idx(outpt_arena, op_start).prev == next {
        return PointInPolygonResult::IsOutside;
    }

    let mut op = op_start;
    loop {
        if idx(outpt_arena, op).pt.y != pt.y {
            break;
        }
        op = idx(outpt_arena, op).next;
        if op == op_start {
            break;
        }
    }
    if idx(outpt_arena, op).pt.y == pt.y {
        return PointInPolygonResult::IsOutside;
    }

    let mut is_above = idx(outpt_arena, op).pt.y < pt.y;
    let starting_above = is_above;
    let mut val = 0;
    let mut op2 = idx(outpt_arena, op).next;

    while op2 != op {
        if is_above {
            while op2 != op && idx(outpt_arena, op2).pt.y < pt.y {
                op2 = idx(outpt_arena, op2).next;
            }
        } else {
            while op2 != op && idx(outpt_arena, op2).pt.y > pt.y {
                op2 = idx(outpt_arena, op2).next;
            }
        }
        if op2 == op {
            break;
        }

        if idx(outpt_arena, op2).pt.y == pt.y {
            let prev = idx(outpt_arena, op2).prev;
            if idx(outpt_arena, op2).pt.x == pt.x
                || (idx(outpt_arena, op2).pt.y == idx(outpt_arena, prev).pt.y
                    && (pt.x < idx(outpt_arena, prev).pt.x) != (pt.x < idx(outpt_arena, op2).pt.x))
            {
                return PointInPolygonResult::IsOn;
            }
            op2 = idx(outpt_arena, op2).next;
            if op2 == op {
                break;
            }
            continue;
        }

        let prev = idx(outpt_arena, op2).prev;
        if pt.x < idx(outpt_arena, op2).pt.x && pt.x < idx(outpt_arena, prev).pt.x {
            // do nothing
        } else if pt.x > idx(outpt_arena, prev).pt.x && pt.x > idx(outpt_arena, op2).pt.x {
            val = 1 - val;
        } else {
            let i = cross_product_sign(idx(outpt_arena, prev).pt, idx(outpt_arena, op2).pt, pt);
            if i == 0 {
                return PointInPolygonResult::IsOn;
            }
            if (i < 0) == is_above {
                val = 1 - val;
            }
        }
        is_above = !is_above;
        op2 = idx(outpt_arena, op2).next;
    }

    if is_above != starting_above {
        let prev = idx(outpt_arena, op2).prev;
        let i = cross_product_sign(idx(outpt_arena, prev).pt, idx(outpt_arena, op2).pt, pt);
        if i == 0 {
            return PointInPolygonResult::IsOn;
        }
        if (i < 0) == is_above {
            val = 1 - val;
        }
    }

    if val == 0 {
        PointInPolygonResult::IsOutside
    } else {
        PointInPolygonResult::IsInside
    }
}

/// Check if path1 (as OutPt list) is contained within path2 (as OutPt list)
/// Direct port from clipper.engine.cpp Path2ContainsPath1 (line 576)
pub fn path2_contains_path1_outpt(op1_start: Idx, op2_start: Idx, outpt_arena: &[OutPt]) -> bool {
    let mut pip = PointInPolygonResult::IsOn;
    let mut op = op1_start;
    loop {
        match point_in_op_polygon(idx(outpt_arena, op).pt, op2_start, outpt_arena) {
            PointInPolygonResult::IsOutside => {
                if pip == PointInPolygonResult::IsOutside {
                    return false;
                }
                pip = PointInPolygonResult::IsOutside;
            }
            PointInPolygonResult::IsInside => {
                if pip == PointInPolygonResult::IsInside {
                    return true;
                }
                pip = PointInPolygonResult::IsInside;
            }
            _ => {}
        }
        op = idx(outpt_arena, op).next;
        if op == op1_start {
            break;
        }
    }
    // result unclear, try using cleaned paths
    let clean1 = get_clean_path(op1_start, outpt_arena);
    let clean2 = get_clean_path(op2_start, outpt_arena);
    path2_contains_path1(&clean1, &clean2)
}

/// Check if path1 is contained within path2 using Path64 vectors
/// Direct port from clipper.h line 717 - template Path2ContainsPath1<T>
/// precondition: paths must not intersect, except for transient micro intersections
pub fn path2_contains_path1(path1: &Path64, path2: &Path64) -> bool {
    let mut pip = PointInPolygonResult::IsOn;
    for pt in path1 {
        match point_in_polygon(*pt, path2) {
            PointInPolygonResult::IsOutside => {
                if pip == PointInPolygonResult::IsOutside {
                    return false;
                }
                pip = PointInPolygonResult::IsOutside;
            }
            PointInPolygonResult::IsInside => {
                if pip == PointInPolygonResult::IsInside {
                    return true;
                }
                pip = PointInPolygonResult::IsInside;
            }
            _ => {}
        }
    }
    if pip != PointInPolygonResult::IsInside {
        return false;
    }
    // result is likely true but check midpoint
    let mp1 = get_bounds_path(path1).mid_point();
    point_in_polygon(mp1, path2) == PointInPolygonResult::IsInside
}

/// Build a Path64 from OutPt circular list
/// Direct port from clipper.engine.cpp BuildPath64 (line 2891)
pub fn build_path64_from_outpt(
    op_start: Idx,
    reverse: bool,
    is_open: bool,
    outpt_arena: &[OutPt],
) -> Option<Path64> {
    let next = idx(outpt_arena, op_start).next;
    if next == op_start || (!is_open && next == idx(outpt_arena, op_start).prev) {
        return None;
    }

    let mut path = Path64::new();
    let (mut last_pt, mut op2);

    if reverse {
        last_pt = idx(outpt_arena, op_start).pt;
        op2 = idx(outpt_arena, op_start).prev;
    } else {
        let op_next = idx(outpt_arena, op_start).next;
        last_pt = idx(outpt_arena, op_next).pt;
        op2 = idx(outpt_arena, op_next).next;
        path.push(last_pt);
        while op2 != idx(outpt_arena, op_start).next {
            if idx(outpt_arena, op2).pt != last_pt {
                last_pt = idx(outpt_arena, op2).pt;
                path.push(last_pt);
            }
            op2 = idx(outpt_arena, op2).next;
        }
        if !is_open
            && path.len() == 3
            && is_very_small_triangle(idx(outpt_arena, op_start).next, outpt_arena)
        {
            return None;
        }
        return if path.len() >= 2 { Some(path) } else { None };
    }

    // reverse case
    path.push(last_pt);
    while op2 != op_start {
        if idx(outpt_arena, op2).pt != last_pt {
            last_pt = idx(outpt_arena, op2).pt;
            path.push(last_pt);
        }
        op2 = idx(outpt_arena, op2).prev;
    }

    if !is_open && path.len() == 3 && is_very_small_triangle(op_start, outpt_arena) {
        return None;
    }

    if path.len() >= 2 {
        Some(path)
    } else {
        None
    }
}

/// Build a PathD from OutPt circular list
/// Direct port from clipper.engine.cpp BuildPathD (line 3055)
pub fn build_path_d_from_outpt(
    op_start: Idx,
    reverse: bool,
    is_open: bool,
    outpt_arena: &[OutPt],
    inv_scale: f64,
) -> Option<PathD> {
    let next = idx(outpt_arena, op_start).next;
    if next == op_start || (!is_open && next == idx(outpt_arena, op_start).prev) {
        return None;
    }

    let mut path = PathD::new();
    let (mut last_pt, mut op2);

    if reverse {
        last_pt = idx(outpt_arena, op_start).pt;
        op2 = idx(outpt_arena, op_start).prev;
    } else {
        let op_next = idx(outpt_arena, op_start).next;
        last_pt = idx(outpt_arena, op_next).pt;
        op2 = idx(outpt_arena, op_next).next;
        path.push(PointD {
            x: last_pt.x as f64 * inv_scale,
            y: last_pt.y as f64 * inv_scale,
            #[cfg(feature = "using_z")]
            z: last_pt.z,
        });
        while op2 != idx(outpt_arena, op_start).next {
            if idx(outpt_arena, op2).pt != last_pt {
                last_pt = idx(outpt_arena, op2).pt;
                path.push(PointD {
                    x: last_pt.x as f64 * inv_scale,
                    y: last_pt.y as f64 * inv_scale,
                    #[cfg(feature = "using_z")]
                    z: last_pt.z,
                });
            }
            op2 = idx(outpt_arena, op2).next;
        }
        if path.len() == 3 && is_very_small_triangle(idx(outpt_arena, op_start).next, outpt_arena) {
            return None;
        }
        return if path.len() >= 2 { Some(path) } else { None };
    }

    // reverse case
    path.push(PointD {
        x: last_pt.x as f64 * inv_scale,
        y: last_pt.y as f64 * inv_scale,
        #[cfg(feature = "using_z")]
        z: last_pt.z,
    });
    while op2 != op_start {
        if idx(outpt_arena, op2).pt != last_pt {
            last_pt = idx(outpt_arena, op2).pt;
            path.push(PointD {
                x: last_pt.x as f64 * inv_scale,
                y: last_pt.y as f64 * inv_scale,
                #[cfg(feature = "using_z")]
                z: last_pt.z,
            });
        }
        op2 = idx(outpt_arena, op2).prev;
    }

    if path.len() == 3 && is_very_small_triangle(op_start, outpt_arena) {
        return None;
    }

    if path.len() >= 2 {
        Some(path)
    } else {
        None
    }
}

/// Get the last output point for a hot edge
/// Direct port from clipper.engine.cpp GetLastOp (line 2496)
#[inline]
pub fn get_last_op(
    hot_edge_idx: Idx,
    active_arena: &[Active],
    outrec_list: &[OutRec],
    outpt_arena: &[OutPt],
) -> Idx {
    let or_idx = idx(active_arena, hot_edge_idx).outrec;
    let result = idx(outrec_list, or_idx).pts;
    if hot_edge_idx != idx(outrec_list, or_idx).front_edge {
        idx(outpt_arena, result).next
    } else {
        result
    }
}

/// Fix all OutPt outrec references in an OutRec
/// Direct port from clipper.engine.cpp FixOutRecPts (line 2158)
pub fn fix_outrec_pts(outrec_idx: Idx, outrec_list: &[OutRec], outpt_arena: &mut [OutPt]) {
    let pts = idx(outrec_list, outrec_idx).pts;
    if pts != NONE {
        let mut op = pts;
        loop {
            let op_pt = idx_mut(outpt_arena, op);
            op_pt.outrec = outrec_idx;
            op = op_pt.next;
            if op == pts {
                break;
            }
        }
    }
}

/// Update all OutPt outrec references in an OutRec
/// Direct port from clipper.engine.cpp UpdateOutrecOwner (line 1684)
pub fn update_outrec_owner(outrec_idx: Idx, outrec_list: &[OutRec], outpt_arena: &mut [OutPt]) {
    fix_outrec_pts(outrec_idx, outrec_list, outpt_arena);
}

/// Move splits from one OutRec to another
/// Direct port from clipper.engine.cpp MoveSplits (line 2269)
pub fn move_splits(outrec_list: &mut [OutRec], from_or: Idx, to_or: Idx) {
    let from_splits = std::mem::take(&mut idx_mut(outrec_list, from_or).splits);
    let to_splits = &mut idx_mut(outrec_list, to_or).splits;
    for s in from_splits {
        if s != to_or {
            to_splits.push(s);
        }
    }
}

/// Uncouple an OutRec from its edges
/// Direct port from clipper.engine.cpp UncoupleOutRec (line 425)
pub fn uncouple_outrec(e_idx: Idx, active_arena: &mut [Active], outrec_list: &mut [OutRec]) {
    let or_idx = idx(active_arena, e_idx).outrec;
    if or_idx != NONE {
        let fe = idx(outrec_list, or_idx).front_edge;
        if fe != NONE {
            idx_mut(active_arena, fe).outrec = NONE;
        }
        let be = idx(outrec_list, or_idx).back_edge;
        if be != NONE {
            idx_mut(active_arena, be).outrec = NONE;
        }
        let or = idx_mut(outrec_list, or_idx);
        or.front_edge = NONE;
        or.back_edge = NONE;
    }
}

/// Extract an active from the SEL (sorted edge list)
/// Direct port from clipper.engine.cpp ExtractFromSEL
pub fn extract_from_sel(e_idx: Idx, active_arena: &mut [Active]) -> Idx {
    let next = idx(active_arena, e_idx).next_in_sel;
    if next != NONE {
        let e_prev = idx(active_arena, e_idx).prev_in_sel;
        idx_mut(active_arena, next).prev_in_sel = e_prev;
    }
    let prev_idx = idx(active_arena, e_idx).prev_in_sel;
    if prev_idx != NONE {
        idx_mut(active_arena, prev_idx).next_in_sel = next;
    }
    let e = idx_mut(active_arena, e_idx);
    e.prev_in_sel = NONE;
    e.next_in_sel = NONE;
    next
}

/// Insert tmp before left in SEL
/// Direct port from clipper.engine.cpp Insert1Before2InSEL
pub fn insert1_before2_in_sel(tmp_idx: Idx, left_idx: Idx, active_arena: &mut [Active]) {
    let prev = idx(active_arena, left_idx).prev_in_sel;
    idx_mut(active_arena, tmp_idx).prev_in_sel = prev;
    if prev != NONE {
        idx_mut(active_arena, prev).next_in_sel = tmp_idx;
    }
    idx_mut(active_arena, tmp_idx).next_in_sel = left_idx;
    idx_mut(active_arena, left_idx).prev_in_sel = tmp_idx;
}
