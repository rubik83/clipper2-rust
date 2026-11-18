//! FAST rectangular clipping implementation
//!
//! Direct port from clipper.rectclip.h and clipper.rectclip.cpp
//! Copyright (c) Angus Johnson 2010-2025
//! Provides high-performance rectangle clipping functionality

use crate::{core::*, idx, idx_mut, Idx, NONE};

// ============================================================================
// OutPt2 - Arena-allocated doubly-linked circular list node
// Direct port from clipper.rectclip.h line 25
// ============================================================================

/// Output point for rectangle clipping, using arena indices instead of raw pointers
/// Direct port from clipper.rectclip.h line 25
struct OutPt2 {
    pt: Point64,
    owner_idx: Idx,
    /// Which edge array (0-7) this point belongs to, or None
    edge_idx: Idx,
    /// Index of next OutPt2 in the arena
    next: Idx,
    /// Index of previous OutPt2 in the arena
    prev: Idx,
}

impl OutPt2 {
    fn new(pt: Point64) -> Self {
        Self {
            pt,
            owner_idx: 0,
            edge_idx: NONE,
            next: 0,
            prev: 0,
        }
    }
}

// ============================================================================
// Free helper functions
// Direct port from clipper.rectclip.cpp lines 19-311
// ============================================================================

/// Check if path1 contains path2
/// Direct port from clipper.rectclip.cpp line 19
fn path1_contains_path2(path1: &Path64, path2: &Path64) -> bool {
    let mut io_count = 0i32;
    for pt in path2 {
        let pip = point_in_polygon(*pt, path1);
        match pip {
            PointInPolygonResult::IsOutside => io_count += 1,
            PointInPolygonResult::IsInside => io_count -= 1,
            _ => continue,
        }
        if io_count.abs() > 1 {
            break;
        }
    }
    io_count <= 0
}

/// Get segment intersection with improved edge-case handling for rect clipping
/// Direct port from clipper.rectclip.cpp line 73
fn get_segment_intersection(
    p1: Point64,
    p2: Point64,
    p3: Point64,
    p4: Point64,
    ip: &mut Point64,
) -> bool {
    let res1 = cross_product_sign(p1, p3, p4);
    let res2 = cross_product_sign(p2, p3, p4);
    if res1 == 0 {
        *ip = p1;
        if res2 == 0 {
            return false; // segments are collinear
        } else if p1 == p3 || p1 == p4 {
            return true;
        } else if is_horizontal(&p3, &p4) {
            return (p1.x > p3.x) == (p1.x < p4.x);
        } else {
            return (p1.y > p3.y) == (p1.y < p4.y);
        }
    } else if res2 == 0 {
        *ip = p2;
        if p2 == p3 || p2 == p4 {
            return true;
        } else if is_horizontal(&p3, &p4) {
            return (p2.x > p3.x) == (p2.x < p4.x);
        } else {
            return (p2.y > p3.y) == (p2.y < p4.y);
        }
    }
    if (res1 > 0) == (res2 > 0) {
        return false;
    }

    let res3 = cross_product_sign(p3, p1, p2);
    let res4 = cross_product_sign(p4, p1, p2);
    if res3 == 0 {
        *ip = p3;
        if p3 == p1 || p3 == p2 {
            return true;
        } else if is_horizontal(&p1, &p2) {
            return (p3.x > p1.x) == (p3.x < p2.x);
        } else {
            return (p3.y > p1.y) == (p3.y < p2.y);
        }
    } else if res4 == 0 {
        *ip = p4;
        if p4 == p1 || p4 == p2 {
            return true;
        } else if is_horizontal(&p1, &p2) {
            return (p4.x > p1.x) == (p4.x < p2.x);
        } else {
            return (p4.y > p1.y) == (p4.y < p2.y);
        }
    }
    if (res3 > 0) == (res4 > 0) {
        return false;
    }

    // segments must intersect to get here
    get_segment_intersect_pt(p1, p2, p3, p4, ip)
}

/// Get intersection of a point-pair with the rect boundary closest to 'p'
/// Direct port from clipper.rectclip.cpp line 118
fn get_intersection(
    rect_path: &Path64,
    p: Point64,
    p2: Point64,
    loc: &mut Location,
    ip: &mut Point64,
) -> bool {
    match *loc {
        Location::Left => {
            if get_segment_intersection(p, p2, rect_path[0], rect_path[3], ip) {
                return true;
            } else if p.y < rect_path[0].y
                && get_segment_intersection(p, p2, rect_path[0], rect_path[1], ip)
            {
                *loc = Location::Top;
                return true;
            } else if get_segment_intersection(p, p2, rect_path[2], rect_path[3], ip) {
                *loc = Location::Bottom;
                return true;
            }
            false
        }
        Location::Top => {
            if get_segment_intersection(p, p2, rect_path[0], rect_path[1], ip) {
                return true;
            } else if p.x < rect_path[0].x
                && get_segment_intersection(p, p2, rect_path[0], rect_path[3], ip)
            {
                *loc = Location::Left;
                return true;
            } else if get_segment_intersection(p, p2, rect_path[1], rect_path[2], ip) {
                *loc = Location::Right;
                return true;
            }
            false
        }
        Location::Right => {
            if get_segment_intersection(p, p2, rect_path[1], rect_path[2], ip) {
                return true;
            } else if p.y < rect_path[1].y
                && get_segment_intersection(p, p2, rect_path[0], rect_path[1], ip)
            {
                *loc = Location::Top;
                return true;
            } else if get_segment_intersection(p, p2, rect_path[2], rect_path[3], ip) {
                *loc = Location::Bottom;
                return true;
            }
            false
        }
        Location::Bottom => {
            if get_segment_intersection(p, p2, rect_path[2], rect_path[3], ip) {
                return true;
            } else if p.x < rect_path[3].x
                && get_segment_intersection(p, p2, rect_path[0], rect_path[3], ip)
            {
                *loc = Location::Left;
                return true;
            } else if get_segment_intersection(p, p2, rect_path[1], rect_path[2], ip) {
                *loc = Location::Right;
                return true;
            }
            false
        }
        Location::Inside => {
            if get_segment_intersection(p, p2, rect_path[0], rect_path[3], ip) {
                *loc = Location::Left;
                return true;
            } else if get_segment_intersection(p, p2, rect_path[0], rect_path[1], ip) {
                *loc = Location::Top;
                return true;
            } else if get_segment_intersection(p, p2, rect_path[1], rect_path[2], ip) {
                *loc = Location::Right;
                return true;
            } else if get_segment_intersection(p, p2, rect_path[2], rect_path[3], ip) {
                *loc = Location::Bottom;
                return true;
            }
            false
        }
    }
}

/// Get adjacent location (clockwise or counter-clockwise)
/// Direct port from clipper.rectclip.cpp line 206
#[inline]
fn get_adjacent_location(loc: Location, is_clockwise: bool) -> Location {
    let delta = if is_clockwise { 1 } else { 3 };
    let idx = (loc as i32 + delta) % 4;
    match idx {
        0 => Location::Left,
        1 => Location::Top,
        2 => Location::Right,
        3 => Location::Bottom,
        _ => unreachable!(),
    }
}

/// Check if heading is clockwise between two locations
/// Direct port from clipper.rectclip.cpp line 212
#[inline]
fn heading_clockwise(prev: Location, curr: Location) -> bool {
    (prev as i32 + 1) % 4 == curr as i32
}

/// Check if two locations are opposite
/// Direct port from clipper.rectclip.cpp line 217
#[inline]
fn are_opposites(prev: Location, curr: Location) -> bool {
    (prev as i32 - curr as i32).abs() == 2
}

/// Determine if the path from prev to curr is clockwise
/// Direct port from clipper.rectclip.cpp line 222
#[inline]
fn is_clockwise_dir(
    prev: Location,
    curr: Location,
    prev_pt: &Point64,
    curr_pt: &Point64,
    rect_mp: &Point64,
) -> bool {
    if are_opposites(prev, curr) {
        cross_product_three_points(*prev_pt, *rect_mp, *curr_pt) < 0.0
    } else {
        heading_clockwise(prev, curr)
    }
}

/// Get which edges of the rectangle the point is on (bitmask)
/// Direct port from clipper.rectclip.cpp line 247
#[inline]
fn get_edges_for_pt(pt: &Point64, rec: &Rect64) -> u32 {
    let mut result = 0u32;
    if pt.x == rec.left {
        result = 1;
    } else if pt.x == rec.right {
        result = 4;
    }
    if pt.y == rec.top {
        result += 2;
    } else if pt.y == rec.bottom {
        result += 8;
    }
    result
}

/// Check if heading is clockwise along a specific edge
/// Direct port from clipper.rectclip.cpp line 257
#[inline]
fn is_heading_clockwise(pt1: &Point64, pt2: &Point64, edge_idx: Idx) -> bool {
    match edge_idx {
        0 => pt2.y < pt1.y,
        1 => pt2.x > pt1.x,
        2 => pt2.y > pt1.y,
        _ => pt2.x < pt1.x,
    }
}

/// Check for horizontal overlap between two segments
/// Direct port from clipper.rectclip.cpp line 268
#[inline]
fn has_horz_overlap(left1: &Point64, right1: &Point64, left2: &Point64, right2: &Point64) -> bool {
    (left1.x < right2.x) && (right1.x > left2.x)
}

/// Check for vertical overlap between two segments
/// Direct port from clipper.rectclip.cpp line 274
#[inline]
fn has_vert_overlap(top1: &Point64, bottom1: &Point64, top2: &Point64, bottom2: &Point64) -> bool {
    (top1.y < bottom2.y) && (bottom1.y > top2.y)
}

/// Check if start locations indicate a clockwise direction
/// Direct port from clipper.rectclip.cpp line 426
fn start_locs_are_clockwise(start_locs: &[Location]) -> bool {
    let mut result = 0i32;
    for i in 1..start_locs.len() {
        let d = start_locs[i] as i32 - start_locs[i - 1] as i32;
        match d {
            -1 => result -= 1,
            1 => result += 1,
            -3 => result += 1,
            3 => result -= 1,
            _ => {}
        }
    }
    result > 0
}

// ============================================================================
// RectClip64 - Main rectangular clipper for polygon clipping
// Direct port from clipper.rectclip.h line 38
// ============================================================================

/// Main rectangular clipper class for polygon clipping
/// Direct port from clipper.rectclip.h line 38
pub struct RectClip64 {
    rect: Rect64,
    rect_as_path: Path64,
    rect_mp: Point64,
    path_bounds: Rect64,
    outpt2_arena: Vec<OutPt2>,
    results: Vec<Idx>,
    edges: [Vec<Idx>; 8],
    start_locs: Vec<Location>,
}

impl RectClip64 {
    /// Create new rectangular clipper
    /// Direct port from clipper.rectclip.h line 59
    pub fn new(rect: Rect64) -> Self {
        let rect_as_path = rect.as_path();
        let rect_mp = rect.mid_point();
        Self {
            rect,
            rect_as_path,
            rect_mp,
            path_bounds: Rect64::new(0, 0, 0, 0),
            outpt2_arena: Vec::new(),
            results: Vec::new(),
            edges: Default::default(),
            start_locs: Vec::new(),
        }
    }

    /// Clear all internal state for reuse
    fn clear(&mut self) {
        self.outpt2_arena.clear();
        self.results.clear();
        for edge in &mut self.edges {
            edge.clear();
        }
        self.start_locs.clear();
    }

    #[inline]
    fn outpt2(&self, i: Idx) -> &OutPt2 {
        idx(&self.outpt2_arena, i)
    }

    #[inline]
    fn outpt2_mut(&mut self, i: Idx) -> &mut OutPt2 {
        idx_mut(&mut self.outpt2_arena, i)
    }

    /// Add a point to the current result path
    /// Direct port from clipper.rectclip.cpp line 317
    fn add(&mut self, pt: Point64, start_new: bool) -> Idx {
        let curr_idx = self.results.len() as Idx;
        if curr_idx == 0 || start_new {
            let new_idx = self.outpt2_arena.len() as Idx;
            let mut op = OutPt2::new(pt);
            op.next = new_idx;
            op.prev = new_idx;
            self.outpt2_arena.push(op);
            self.results.push(new_idx);
            new_idx
        } else {
            let result_idx = curr_idx - 1;
            let prev_op_idx = *idx(&self.results, result_idx);
            if self.outpt2(prev_op_idx).pt == pt {
                return prev_op_idx;
            }
            let new_idx = self.outpt2_arena.len() as Idx;
            let mut op = OutPt2::new(pt);
            op.owner_idx = result_idx;

            // Insert after prev_op in the circular list
            let prev_next = self.outpt2(prev_op_idx).next;
            op.next = prev_next;
            op.prev = prev_op_idx;
            self.outpt2_arena.push(op);

            self.outpt2_mut(prev_next).prev = new_idx;
            self.outpt2_mut(prev_op_idx).next = new_idx;

            *idx_mut(&mut self.results, result_idx) = new_idx;
            new_idx
        }
    }

    /// Add a corner of the rectangle
    /// Direct port from clipper.rectclip.cpp line 349
    fn add_corner_prev_curr(&mut self, prev: Location, curr: Location) {
        if heading_clockwise(prev, curr) {
            self.add(*idx(&self.rect_as_path, prev as Idx), false);
        } else {
            self.add(*idx(&self.rect_as_path, curr as Idx), false);
        }
    }

    /// Add a corner and advance location
    /// Direct port from clipper.rectclip.cpp line 357
    fn add_corner_loc(&mut self, loc: &mut Location, is_clockwise: bool) {
        if is_clockwise {
            let pt = *idx(&self.rect_as_path, *loc as Idx);
            self.add(pt, false);
            *loc = get_adjacent_location(*loc, true);
        } else {
            *loc = get_adjacent_location(*loc, false);
            let pt = *idx(&self.rect_as_path, *loc as Idx);
            self.add(pt, false);
        }
    }

    /// Get next location along the path
    /// Direct port from clipper.rectclip.cpp line 371
    fn get_next_location(
        &mut self,
        path: &Path64,
        loc: &mut Location,
        i: &mut usize,
        high_i: usize,
    ) {
        match *loc {
            Location::Left => {
                while *i <= high_i && path[*i].x <= self.rect.left {
                    *i += 1;
                }
                if *i > high_i {
                    return;
                }
                if path[*i].x >= self.rect.right {
                    *loc = Location::Right;
                } else if path[*i].y <= self.rect.top {
                    *loc = Location::Top;
                } else if path[*i].y >= self.rect.bottom {
                    *loc = Location::Bottom;
                } else {
                    *loc = Location::Inside;
                }
            }
            Location::Top => {
                while *i <= high_i && path[*i].y <= self.rect.top {
                    *i += 1;
                }
                if *i > high_i {
                    return;
                }
                if path[*i].y >= self.rect.bottom {
                    *loc = Location::Bottom;
                } else if path[*i].x <= self.rect.left {
                    *loc = Location::Left;
                } else if path[*i].x >= self.rect.right {
                    *loc = Location::Right;
                } else {
                    *loc = Location::Inside;
                }
            }
            Location::Right => {
                while *i <= high_i && path[*i].x >= self.rect.right {
                    *i += 1;
                }
                if *i > high_i {
                    return;
                }
                if path[*i].x <= self.rect.left {
                    *loc = Location::Left;
                } else if path[*i].y <= self.rect.top {
                    *loc = Location::Top;
                } else if path[*i].y >= self.rect.bottom {
                    *loc = Location::Bottom;
                } else {
                    *loc = Location::Inside;
                }
            }
            Location::Bottom => {
                while *i <= high_i && path[*i].y >= self.rect.bottom {
                    *i += 1;
                }
                if *i > high_i {
                    return;
                }
                if path[*i].y <= self.rect.top {
                    *loc = Location::Top;
                } else if path[*i].x <= self.rect.left {
                    *loc = Location::Left;
                } else if path[*i].x >= self.rect.right {
                    *loc = Location::Right;
                } else {
                    *loc = Location::Inside;
                }
            }
            Location::Inside => {
                while *i <= high_i {
                    if path[*i].x < self.rect.left {
                        *loc = Location::Left;
                    } else if path[*i].x > self.rect.right {
                        *loc = Location::Right;
                    } else if path[*i].y > self.rect.bottom {
                        *loc = Location::Bottom;
                    } else if path[*i].y < self.rect.top {
                        *loc = Location::Top;
                    } else {
                        let pt = path[*i];
                        self.add(pt, false);
                        *i += 1;
                        continue;
                    }
                    break;
                }
            }
        }
    }

    /// Unlink a node from the circular list, return the next node (or None if list becomes empty)
    /// Direct port from clipper.rectclip.cpp line 231
    fn unlink_op(&mut self, op_idx: Idx) -> Idx {
        let next = self.outpt2(op_idx).next;
        if next == op_idx {
            return NONE;
        }
        let prev = self.outpt2(op_idx).prev;
        self.outpt2_mut(prev).next = next;
        self.outpt2_mut(next).prev = prev;
        next
    }

    /// Unlink a node from the circular list, return the prev node (or None if list becomes empty)
    /// Direct port from clipper.rectclip.cpp line 239
    fn unlink_op_back(&mut self, op_idx: Idx) -> Idx {
        let next = self.outpt2(op_idx).next;
        if next == op_idx {
            return NONE;
        }
        let prev = self.outpt2(op_idx).prev;
        self.outpt2_mut(prev).next = next;
        self.outpt2_mut(next).prev = prev;
        prev
    }

    /// Add a point to an edge list
    /// Direct port from clipper.rectclip.cpp line 280
    fn add_to_edge(&mut self, edge_idx: Idx, op_idx: Idx) {
        if self.outpt2(op_idx).edge_idx != NONE {
            return;
        }
        self.outpt2_mut(op_idx).edge_idx = edge_idx;
        idx_mut(&mut self.edges, edge_idx).push(op_idx);
    }

    /// Remove a point from its edge list
    /// Direct port from clipper.rectclip.cpp line 287
    fn uncouple_edge(&mut self, op_idx: Idx) {
        let edge_idx = idx(&self.outpt2_arena, op_idx).edge_idx;
        if edge_idx == NONE {
            return;
        }
        if let Some(idx) = idx_mut(&mut self.edges, edge_idx)
            .iter_mut()
            .find(|&&mut idx| idx == op_idx)
        {
            *idx = NONE;
        }
        self.outpt2_mut(op_idx).edge_idx = NONE;
    }

    /// Set new owner for all nodes in a circular list
    /// Direct port from clipper.rectclip.cpp line 302
    fn set_new_owner(&mut self, op_idx: Idx, new_idx: Idx) {
        self.outpt2_mut(op_idx).owner_idx = new_idx;
        let mut op2 = self.outpt2(op_idx).next;
        while op2 != op_idx {
            self.outpt2_mut(op2).owner_idx = new_idx;
            op2 = self.outpt2(op2).next;
        }
    }

    /// Internal execution for polygon clipping against a single path
    /// Direct port from clipper.rectclip.cpp line 443
    fn execute_internal(&mut self, path: &Path64) {
        if path.is_empty() {
            return;
        }

        let high_i = path.len() - 1;
        let mut prev = Location::Inside;
        let mut loc = Location::Inside;
        let mut crossing_loc = Location::Inside;
        let mut first_cross = Location::Inside;

        if !get_location(&self.rect, &path[high_i], &mut loc) {
            let mut i = high_i;
            while i > 0 && !get_location(&self.rect, &path[i - 1], &mut prev) {
                i -= 1;
            }
            if i == 0 {
                // all of path must be inside rect
                for pt in path {
                    self.add(*pt, false);
                }
                return;
            }
            if prev == Location::Inside {
                loc = Location::Inside;
            }
        }
        let starting_loc = loc;

        let mut i = 0usize;
        while i <= high_i {
            prev = loc;
            let crossing_prev = crossing_loc;

            self.get_next_location(path, &mut loc, &mut i, high_i);

            if i > high_i {
                break;
            }
            let mut ip = Point64::new(0, 0);
            let mut ip2 = Point64::new(0, 0);
            let prev_pt = if i > 0 { path[i - 1] } else { path[high_i] };

            crossing_loc = loc;
            if !get_intersection(
                &self.rect_as_path.clone(),
                path[i],
                prev_pt,
                &mut crossing_loc,
                &mut ip,
            ) {
                // ie remaining outside
                if crossing_prev == Location::Inside {
                    let is_clockw = is_clockwise_dir(prev, loc, &prev_pt, &path[i], &self.rect_mp);
                    let mut p = prev;
                    loop {
                        self.start_locs.push(p);
                        p = get_adjacent_location(p, is_clockw);
                        if p == loc {
                            break;
                        }
                    }
                    crossing_loc = crossing_prev; // still not crossed
                } else if prev != Location::Inside && prev != loc {
                    let is_clockw = is_clockwise_dir(prev, loc, &prev_pt, &path[i], &self.rect_mp);
                    let mut p = prev;
                    loop {
                        self.add_corner_loc(&mut p, is_clockw);
                        if p == loc {
                            break;
                        }
                    }
                }
                i += 1;
                continue;
            }

            // we must be crossing the rect boundary to get here

            if loc == Location::Inside {
                // path must be entering rect
                if first_cross == Location::Inside {
                    first_cross = crossing_loc;
                    self.start_locs.push(prev);
                } else if prev != crossing_loc {
                    let is_clockw =
                        is_clockwise_dir(prev, crossing_loc, &prev_pt, &path[i], &self.rect_mp);
                    let mut p = prev;
                    loop {
                        self.add_corner_loc(&mut p, is_clockw);
                        if p == crossing_loc {
                            break;
                        }
                    }
                }
            } else if prev != Location::Inside {
                // passing right through rect
                loc = prev;
                let rect_as_path = self.rect_as_path.clone();
                get_intersection(&rect_as_path, prev_pt, path[i], &mut loc, &mut ip2);

                if crossing_prev != Location::Inside && crossing_prev != loc {
                    self.add_corner_prev_curr(crossing_prev, loc);
                }

                if first_cross == Location::Inside {
                    first_cross = loc;
                    self.start_locs.push(prev);
                }

                loc = crossing_loc;
                self.add(ip2, false);
                if ip == ip2 {
                    // it's very likely that path[i] is on rect
                    get_location(&self.rect, &path[i], &mut loc);
                    self.add_corner_prev_curr(crossing_loc, loc);
                    crossing_loc = loc;
                    continue;
                }
            } else {
                // path must be exiting rect
                loc = crossing_loc;
                if first_cross == Location::Inside {
                    first_cross = crossing_loc;
                }
            }

            self.add(ip, false);
        } // while i <= high_i

        if first_cross == Location::Inside {
            // path never intersects
            if starting_loc != Location::Inside {
                // path is outside rect but may contain it
                if self.path_bounds.contains_point(&self.rect_mp)
                    && path1_contains_path2(path, &self.rect_as_path)
                {
                    let is_clockwise_path = start_locs_are_clockwise(&self.start_locs);
                    for j in 0..4 {
                        let k = if is_clockwise_path { j } else { 3 - j };
                        let pt = *idx(&self.rect_as_path, k);
                        self.add(pt, false);
                        let results_0 = self.results[0];
                        self.add_to_edge(k * 2, results_0);
                    }
                }
            }
        } else if loc != Location::Inside && (loc != first_cross || self.start_locs.len() > 2) {
            if !self.start_locs.is_empty() {
                let mut p = loc;
                let start_locs_clone = self.start_locs.clone();
                for &loc2 in &start_locs_clone {
                    if p == loc2 {
                        continue;
                    }
                    // C++ calls AddCorner(prev, HeadingClockwise(prev, loc2))
                    // which is the (Location&, bool) overload, not (Location, Location)
                    let hcw = heading_clockwise(p, loc2);
                    self.add_corner_loc(&mut p, hcw);
                    p = loc2;
                }
                loc = p;
            }
            if loc != first_cross {
                // C++ calls AddCorner(loc, HeadingClockwise(loc, first_cross_))
                // which is the (Location&, bool) overload
                let hcw = heading_clockwise(loc, first_cross);
                self.add_corner_loc(&mut loc, hcw);
            }
        }
    }

    /// Check edges after internal execution
    /// Direct port from clipper.rectclip.cpp line 606
    fn check_edges(&mut self) {
        for i in 0..(self.results.len() as Idx) {
            let mut op_idx = *idx_mut(&mut self.results, i);
            if op_idx == NONE {
                continue;
            };

            // Remove collinear points
            let mut op2_idx = op_idx;
            loop {
                let prev_idx = self.outpt2(op2_idx).prev;
                let next_idx = self.outpt2(op2_idx).next;
                let prev_pt = self.outpt2(prev_idx).pt;
                let op2_pt = self.outpt2(op2_idx).pt;
                let next_pt = self.outpt2(next_idx).pt;

                if is_collinear(prev_pt, op2_pt, next_pt) {
                    if op2_idx == op_idx {
                        op2_idx = self.unlink_op_back(op2_idx);
                        if op2_idx != NONE {
                            op_idx = self.outpt2(op2_idx).prev;
                        } else {
                            break;
                        }
                    } else {
                        op2_idx = self.unlink_op_back(op2_idx);
                        if op2_idx != NONE {
                        } else {
                            break;
                        }
                    }
                } else {
                    op2_idx = self.outpt2(op2_idx).next;
                }

                if op2_idx == op_idx {
                    break;
                }
            }

            if op2_idx == NONE {
                *idx_mut(&mut self.results, i) = NONE;
                continue;
            }
            *idx_mut(&mut self.results, i) = op_idx;

            // Assign edges
            let prev_idx = self.outpt2(op_idx).prev;
            let mut edge_set1 = get_edges_for_pt(&self.outpt2(prev_idx).pt, &self.rect);
            let mut op2_idx = op_idx;
            loop {
                let edge_set2 = get_edges_for_pt(&self.outpt2(op2_idx).pt, &self.rect);
                if edge_set2 != 0 && self.outpt2(op2_idx).edge_idx == NONE {
                    let combined_set = edge_set1 & edge_set2;
                    for j in 0..4 {
                        if combined_set & (1 << j) != 0 {
                            let prev_idx = self.outpt2(op2_idx).prev;
                            let prev_pt = self.outpt2(prev_idx).pt;
                            let op2_pt = self.outpt2(op2_idx).pt;
                            if is_heading_clockwise(&prev_pt, &op2_pt, j) {
                                self.add_to_edge(j * 2, op2_idx);
                            } else {
                                self.add_to_edge(j * 2 + 1, op2_idx);
                            }
                        }
                    }
                }
                edge_set1 = edge_set2;
                op2_idx = self.outpt2(op2_idx).next;

                if op2_idx == op_idx {
                    break;
                }
            }
        }
    }

    /// Tidy edges by merging/splitting where cw and ccw edges overlap
    /// Direct port from clipper.rectclip.cpp line 665
    fn tidy_edges(&mut self, cur_idx: Idx, cw_idx: Idx, ccw_idx: Idx) {
        if idx(&self.edges, ccw_idx).is_empty() {
            return;
        }

        let is_horz = cur_idx == 1 || cur_idx == 3;
        let cw_is_toward_larger = cur_idx == 1 || cur_idx == 2;
        let mut i = 0;
        let mut j = 0;

        while i < (idx_mut(&mut self.edges, cw_idx).len() as Idx) {
            let p1_root = *idx(idx(&self.edges, cw_idx).as_slice(), i);
            if p1_root == NONE {
                i += 1;
                j = 0;
                continue;
            }

            // Check if degenerate (next == prev)
            if self.outpt2(p1_root).next == self.outpt2(p1_root).prev {
                *idx_mut(&mut *idx_mut(&mut self.edges, cw_idx), i) = NONE;
                i += 1;
                j = 0;
                continue;
            }

            let j_lim = idx(&self.edges, ccw_idx).len() as Idx;
            while j < j_lim {
                let idx = *idx(idx(&self.edges, ccw_idx).as_slice(), j);
                if idx != NONE {
                    if self.outpt2(idx).next != self.outpt2(idx).prev {
                        break;
                    } else {
                        j += 1
                    }
                }
            }

            if j == j_lim {
                i += 1;
                j = 0;
                continue;
            }

            let p2_root = *idx(idx(&self.edges, ccw_idx).as_slice(), j);

            let (p1, p1a, p2, p2a);
            if cw_is_toward_larger {
                p1 = self.outpt2(p1_root).prev;
                p1a = p1_root;
                p2 = p2_root;
                p2a = self.outpt2(p2_root).prev;
            } else {
                p1 = p1_root;
                p1a = self.outpt2(p1_root).prev;
                p2 = self.outpt2(p2_root).prev;
                p2a = p2_root;
            }

            let p1_pt = self.outpt2(p1).pt;
            let p1a_pt = self.outpt2(p1a).pt;
            let p2_pt = self.outpt2(p2).pt;
            let p2a_pt = self.outpt2(p2a).pt;

            if (is_horz && !has_horz_overlap(&p1_pt, &p1a_pt, &p2_pt, &p2a_pt))
                || (!is_horz && !has_vert_overlap(&p1_pt, &p1a_pt, &p2_pt, &p2a_pt))
            {
                j += 1;
                continue;
            }

            // To get here we're either splitting or rejoining
            let is_rejoining = self.outpt2(p1_root).owner_idx != self.outpt2(p2_root).owner_idx;

            if is_rejoining {
                let p2_owner = self.outpt2(p2).owner_idx;
                let p1_owner = self.outpt2(p1).owner_idx;
                *idx_mut(&mut self.results, p2_owner) = NONE;
                self.set_new_owner(p2, p1_owner);
            }

            // Do the split or re-join
            if cw_is_toward_larger {
                self.outpt2_mut(p1).next = p2;
                self.outpt2_mut(p2).prev = p1;
                self.outpt2_mut(p1a).prev = p2a;
                self.outpt2_mut(p2a).next = p1a;
            } else {
                self.outpt2_mut(p1).prev = p2;
                self.outpt2_mut(p2).next = p1;
                self.outpt2_mut(p1a).next = p2a;
                self.outpt2_mut(p2a).prev = p1a;
            }

            if !is_rejoining {
                let new_idx = self.results.len() as Idx;
                self.results.push(p1a);
                self.set_new_owner(p1a, new_idx);
            }

            let (op, op2);
            if cw_is_toward_larger {
                op = p2;
                op2 = p1a;
            } else {
                op = p1;
                op2 = p2a;
            }

            let op_owner = self.outpt2(op).owner_idx;
            let op2_owner = self.outpt2(op2).owner_idx;
            *idx_mut(&mut self.results, op_owner) = op;
            *idx_mut(&mut self.results, op2_owner) = op2;

            // Get ready for the next loop
            let (op_is_larger, op2_is_larger) = if is_horz {
                let op_prev = self.outpt2(op).prev;
                let op2_prev = self.outpt2(op2).prev;
                (
                    self.outpt2(op).pt.x > self.outpt2(op_prev).pt.x,
                    self.outpt2(op2).pt.x > self.outpt2(op2_prev).pt.x,
                )
            } else {
                let op_prev = self.outpt2(op).prev;
                let op2_prev = self.outpt2(op2).prev;
                (
                    self.outpt2(op).pt.y > self.outpt2(op_prev).pt.y,
                    self.outpt2(op2).pt.y > self.outpt2(op2_prev).pt.y,
                )
            };

            let op_next = self.outpt2(op).next;
            let op_prev = self.outpt2(op).prev;
            let op2_next = self.outpt2(op2).next;
            let op2_prev = self.outpt2(op2).prev;

            if (op_next == op_prev) || (self.outpt2(op).pt == self.outpt2(op_prev).pt) {
                if op2_is_larger == cw_is_toward_larger {
                    *idx_mut(idx_mut(&mut self.edges, cw_idx).as_mut_slice(), i) = op2;
                    *idx_mut(idx_mut(&mut self.edges, ccw_idx).as_mut_slice(), j) = NONE;
                    j += 1;
                } else {
                    *idx_mut(idx_mut(&mut self.edges, ccw_idx).as_mut_slice(), j) = op2;
                    *idx_mut(idx_mut(&mut self.edges, cw_idx).as_mut_slice(), i) = NONE;
                    i += 1;
                }
            } else if (op2_next == op2_prev) || (self.outpt2(op2).pt == self.outpt2(op2_prev).pt) {
                if op_is_larger == cw_is_toward_larger {
                    *idx_mut(idx_mut(&mut self.edges, cw_idx).as_mut_slice(), i) = op;
                    *idx_mut(idx_mut(&mut self.edges, ccw_idx).as_mut_slice(), j) = NONE;
                    j += 1;
                } else {
                    *idx_mut(idx_mut(&mut self.edges, ccw_idx).as_mut_slice(), j) = op;
                    *idx_mut(idx_mut(&mut self.edges, cw_idx).as_mut_slice(), i) = NONE;
                    i += 1;
                }
            } else if op_is_larger == op2_is_larger {
                if op_is_larger == cw_is_toward_larger {
                    *idx_mut(idx_mut(&mut self.edges, cw_idx).as_mut_slice(), i) = op;
                    self.uncouple_edge(op2);
                    self.add_to_edge(cw_idx, op2);
                    *idx_mut(idx_mut(&mut self.edges, ccw_idx).as_mut_slice(), j) = NONE;
                    j += 1;
                } else {
                    *idx_mut(idx_mut(&mut self.edges, cw_idx).as_mut_slice(), i) = NONE;
                    i += 1;
                    *idx_mut(idx_mut(&mut self.edges, ccw_idx).as_mut_slice(), j) = op2;
                    self.uncouple_edge(op);
                    self.add_to_edge(ccw_idx, op);
                    j = 0;
                }
            } else {
                if op_is_larger == cw_is_toward_larger {
                    *idx_mut(idx_mut(&mut self.edges, cw_idx).as_mut_slice(), i) = op;
                } else {
                    *idx_mut(idx_mut(&mut self.edges, ccw_idx).as_mut_slice(), j) = op;
                }
                if op2_is_larger == cw_is_toward_larger {
                    *idx_mut(idx_mut(&mut self.edges, cw_idx).as_mut_slice(), i) = op2;
                } else {
                    *idx_mut(idx_mut(&mut self.edges, ccw_idx).as_mut_slice(), j) = op2;
                }
            }
        }
    }

    /// Extract a path from the circular linked list
    /// Direct port from clipper.rectclip.cpp line 843
    fn get_path(&mut self, op_idx_ref: &mut Idx) -> Path64 {
        let op_start = *op_idx_ref;
        if op_start == NONE {
            return Path64::new();
        }

        // Check if degenerate
        if self.outpt2(op_start).next == self.outpt2(op_start).prev {
            *op_idx_ref = NONE;
            return Path64::new();
        }

        // Remove collinear points
        let mut op_idx = op_start;
        let mut op2_idx = self.outpt2(op_start).next;
        while op2_idx != op_idx {
            let prev_idx = self.outpt2(op2_idx).prev;
            let next_idx = self.outpt2(op2_idx).next;
            let prev_pt = self.outpt2(prev_idx).pt;
            let op2_pt = self.outpt2(op2_idx).pt;
            let next_pt = self.outpt2(next_idx).pt;

            if is_collinear(prev_pt, op2_pt, next_pt) {
                op_idx = self.outpt2(op2_idx).prev;
                let new_idx = self.unlink_op(op2_idx);
                if new_idx != NONE {
                    op2_idx = new_idx;
                } else {
                    *op_idx_ref = NONE;
                    return Path64::new();
                }
            } else {
                op2_idx = self.outpt2(op2_idx).next;
            }
        }

        *op_idx_ref = op2_idx;
        if self.outpt2(op2_idx).next == self.outpt2(op2_idx).prev {
            *op_idx_ref = NONE;
            return Path64::new();
        }

        let mut result = Path64::new();
        let start = op2_idx;
        result.push(self.outpt2(start).pt);
        let mut curr = self.outpt2(start).next;
        while curr != start {
            result.push(self.outpt2(curr).pt);
            curr = self.outpt2(curr).next;
        }
        result
    }

    /// Execute clipping operation on multiple paths
    /// Direct port from clipper.rectclip.cpp line 873
    pub fn execute(&mut self, paths: &Paths64) -> Paths64 {
        let mut result = Paths64::new();
        if self.rect.is_empty() {
            return result;
        }

        for path in paths {
            if path.len() < 3 {
                continue;
            }
            self.path_bounds = get_bounds_path(path);
            if !self.rect.intersects(&self.path_bounds) {
                continue;
            } else if self.rect.contains_rect(&self.path_bounds) {
                result.push(path.clone());
                continue;
            }

            self.execute_internal(path);
            self.check_edges();
            for edge_i in 0..4 {
                self.tidy_edges(edge_i, edge_i * 2, edge_i * 2 + 1);
            }

            for ri in 0..self.results.len() {
                let mut op_ref = self.results[ri];
                let tmp = self.get_path(&mut op_ref);
                self.results[ri] = op_ref;
                if !tmp.is_empty() {
                    result.push(tmp);
                }
            }

            // Clean up after every loop
            self.clear();
        }
        result
    }
}

// ============================================================================
// RectClipLines64 - Rectangular line clipper for line segments
// Direct port from clipper.rectclip.h line 70
// ============================================================================

/// Rectangular line clipper class for line segment clipping
/// Direct port from clipper.rectclip.h line 70
pub struct RectClipLines64 {
    rect: Rect64,
    rect_as_path: Path64,
    #[allow(dead_code)]
    rect_mp: Point64,
    outpt2_arena: Vec<OutPt2>,
    results: Vec<Idx>,
    start_locs: Vec<Location>,
}

impl RectClipLines64 {
    /// Create new line clipper
    /// Direct port from clipper.rectclip.h line 75
    pub fn new(rect: Rect64) -> Self {
        let rect_as_path = rect.as_path();
        let rect_mp = rect.mid_point();
        Self {
            rect,
            rect_as_path,
            rect_mp,
            outpt2_arena: Vec::new(),
            results: Vec::new(),
            start_locs: Vec::new(),
        }
    }

    #[inline]
    fn outpt2(&self, i: Idx) -> &OutPt2 {
        idx(&self.outpt2_arena, i)
    }

    #[inline]
    fn outpt2_mut(&mut self, i: Idx) -> &mut OutPt2 {
        idx_mut(&mut self.outpt2_arena, i)
    }

    fn clear(&mut self) {
        self.outpt2_arena.clear();
        self.results.clear();
        self.start_locs.clear();
    }

    /// Add a point (same logic as RectClip64::Add)
    fn add(&mut self, pt: Point64, start_new: bool) -> Idx {
        let curr_idx = self.results.len() as Idx;
        if curr_idx == 0 || start_new {
            let new_idx = self.outpt2_arena.len() as Idx;
            let mut op = OutPt2::new(pt);
            op.next = new_idx;
            op.prev = new_idx;
            self.outpt2_arena.push(op);
            self.results.push(new_idx);
            new_idx
        } else {
            let result_idx = curr_idx - 1;
            let prev_op_idx = *idx(&self.results, result_idx);
            if self.outpt2(prev_op_idx).pt == pt {
                return prev_op_idx;
            }
            let new_idx = self.outpt2_arena.len() as Idx;
            let mut op = OutPt2::new(pt);
            op.owner_idx = result_idx;

            let prev_next = self.outpt2(prev_op_idx).next;
            op.next = prev_next;
            op.prev = prev_op_idx;
            self.outpt2_arena.push(op);

            self.outpt2_mut(prev_next).prev = new_idx;
            self.outpt2_mut(prev_op_idx).next = new_idx;

            *idx_mut(&mut self.results, result_idx) = new_idx;
            new_idx
        }
    }

    /// Get next location (same logic as RectClip64::GetNextLocation but without add for Inside)
    fn get_next_location(
        &mut self,
        path: &Path64,
        loc: &mut Location,
        i: &mut usize,
        high_i: usize,
    ) {
        match *loc {
            Location::Left => {
                while *i <= high_i && path[*i].x <= self.rect.left {
                    *i += 1;
                }
                if *i > high_i {
                    return;
                }
                if path[*i].x >= self.rect.right {
                    *loc = Location::Right;
                } else if path[*i].y <= self.rect.top {
                    *loc = Location::Top;
                } else if path[*i].y >= self.rect.bottom {
                    *loc = Location::Bottom;
                } else {
                    *loc = Location::Inside;
                }
            }
            Location::Top => {
                while *i <= high_i && path[*i].y <= self.rect.top {
                    *i += 1;
                }
                if *i > high_i {
                    return;
                }
                if path[*i].y >= self.rect.bottom {
                    *loc = Location::Bottom;
                } else if path[*i].x <= self.rect.left {
                    *loc = Location::Left;
                } else if path[*i].x >= self.rect.right {
                    *loc = Location::Right;
                } else {
                    *loc = Location::Inside;
                }
            }
            Location::Right => {
                while *i <= high_i && path[*i].x >= self.rect.right {
                    *i += 1;
                }
                if *i > high_i {
                    return;
                }
                if path[*i].x <= self.rect.left {
                    *loc = Location::Left;
                } else if path[*i].y <= self.rect.top {
                    *loc = Location::Top;
                } else if path[*i].y >= self.rect.bottom {
                    *loc = Location::Bottom;
                } else {
                    *loc = Location::Inside;
                }
            }
            Location::Bottom => {
                while *i <= high_i && path[*i].y >= self.rect.bottom {
                    *i += 1;
                }
                if *i > high_i {
                    return;
                }
                if path[*i].y <= self.rect.top {
                    *loc = Location::Top;
                } else if path[*i].x <= self.rect.left {
                    *loc = Location::Left;
                } else if path[*i].x >= self.rect.right {
                    *loc = Location::Right;
                } else {
                    *loc = Location::Inside;
                }
            }
            Location::Inside => {
                while *i <= high_i {
                    if path[*i].x < self.rect.left {
                        *loc = Location::Left;
                    } else if path[*i].x > self.rect.right {
                        *loc = Location::Right;
                    } else if path[*i].y > self.rect.bottom {
                        *loc = Location::Bottom;
                    } else if path[*i].y < self.rect.top {
                        *loc = Location::Top;
                    } else {
                        let pt = path[*i];
                        self.add(pt, false);
                        *i += 1;
                        continue;
                    }
                    break;
                }
            }
        }
    }

    /// Internal execution for line clipping
    /// Direct port from clipper.rectclip.cpp line 942
    fn execute_internal(&mut self, path: &Path64) {
        if self.rect.is_empty() || path.len() < 2 {
            return;
        }

        self.clear();

        let high_i = path.len() - 1;
        let mut i = 1usize;
        let mut prev = Location::Inside;
        let mut loc = Location::Inside;

        if !get_location(&self.rect, &path[0], &mut loc) {
            while i <= high_i && !get_location(&self.rect, &path[i], &mut prev) {
                i += 1;
            }
            if i > high_i {
                // all of path must be inside rect
                for pt in path {
                    self.add(*pt, false);
                }
                return;
            }
            if prev == Location::Inside {
                loc = Location::Inside;
            }
            i = 1;
        }
        if loc == Location::Inside {
            self.add(path[0], false);
        }

        while i <= high_i {
            prev = loc;
            self.get_next_location(path, &mut loc, &mut i, high_i);
            if i > high_i {
                break;
            }
            let mut ip = Point64::new(0, 0);
            let mut ip2 = Point64::new(0, 0);
            let prev_pt = path[i - 1];

            let mut crossing_loc = loc;
            if !get_intersection(
                &self.rect_as_path.clone(),
                path[i],
                prev_pt,
                &mut crossing_loc,
                &mut ip,
            ) {
                i += 1;
                continue;
            }

            if loc == Location::Inside {
                // path must be entering rect
                self.add(ip, true);
            } else if prev != Location::Inside {
                // passing right through rect
                crossing_loc = prev;
                let rect_as_path = self.rect_as_path.clone();
                get_intersection(&rect_as_path, prev_pt, path[i], &mut crossing_loc, &mut ip2);
                self.add(ip2, true);
                self.add(ip, false);
            } else {
                // path must be exiting rect
                self.add(ip, false);
            }
        }
    }

    /// Extract a path from the circular linked list (line version)
    /// Direct port from clipper.rectclip.cpp line 1012
    fn get_path(&self, op_idx_ref: &mut Idx) -> Path64 {
        let op_start = *op_idx_ref;
        if op_start == NONE {
            return Path64::new();
        }

        if self.outpt2(op_start).next == op_start {
            return Path64::new();
        }

        // Start at path beginning (next from start sentinel)
        let start = self.outpt2(op_start).next;
        let mut result = Path64::new();
        result.push(self.outpt2(start).pt);
        let mut op2 = self.outpt2(start).next;
        while op2 != start {
            result.push(self.outpt2(op2).pt);
            op2 = self.outpt2(op2).next;
        }
        result
    }

    /// Execute line clipping
    /// Direct port from clipper.rectclip.cpp line 916
    pub fn execute(&mut self, paths: &Paths64) -> Paths64 {
        let mut result = Paths64::new();
        if self.rect.is_empty() {
            return result;
        }

        for path in paths {
            let path_rec = get_bounds_path(path);
            if !self.rect.intersects(&path_rec) {
                continue;
            }

            self.execute_internal(path);

            for ri in 0..self.results.len() {
                let mut op_ref = self.results[ri];
                let tmp = self.get_path(&mut op_ref);
                if !tmp.is_empty() {
                    result.push(tmp);
                }
            }
            self.clear();
        }
        result
    }
}

// Include tests from separate file
#[cfg(test)]
#[path = "rectclip_tests.rs"]
mod tests;
