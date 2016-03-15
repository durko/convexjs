var ON = 0;
var LEFT = 1;
var RIGHT = 2;
var ALMOST_ZERO = 0.00001;

function GetSideOfLine(lineStart, lineEnd, point)
{
    var d = (lineEnd[0]-lineStart[0])*(point[1]-lineStart[1])-(lineEnd[1]-lineStart[1])*(point[0]-lineStart[0]);
    return (d > ALMOST_ZERO ? LEFT : (d < -ALMOST_ZERO ? RIGHT : ON));
}

function CalcConvexHull(points)
{
    if (points.length < 3)
        return points;

    var hullPt = points[0];
    var convexHull = [];

    for (var i=1; i<points.length; i++)
    {
        if (points[i][0] < hullPt[0])
            hullPt = points[i];
        else if (Math.abs(points[i][0]-hullPt[0]) < ALMOST_ZERO)
            if (points[i][1] < hullPt[1])
                hullPt = points[i];
    }

    do
    {
        convexHull.unshift([hullPt[0], hullPt[1]]);
        var endPt = points[0];

        for (var j=1; j<points.length; j++)
        {
            var side = GetSideOfLine(hullPt, endPt, points[j]);

            const d1 =
                (hullPt[0]-points[j][0])**2 +
                (hullPt[1]-points[j][1])**2;
            const d2 =
                (hullPt[0]-endPt[0])**2 +
                (hullPt[1]-endPt[1])**2;


            if ((endPt[0] == hullPt[0] && endPt[1] == hullPt[1]) || (side == LEFT || (side == ON && d1 > d2)))
                endPt = points[j];
        }

        hullPt = endPt;
    }
    while (
            endPt[0] != convexHull[convexHull.length-1][0] ||
            endPt[1] != convexHull[convexHull.length-1][1]
            );
    return convexHull;
}

const [TURN_LEFT, TURN_RIGHT, TURN_NONE] = [1, -1, 0];

function sign(value) {
    if (value < 0) {
        return TURN_RIGHT;
    } else if (value == 0) {
        return TURN_NONE;
    } else {
        return TURN_LEFT;
    }
}

function turn(p, q, r) {
    return sign((q[0]-p[0]) * (r[1]-p[1]) - (r[0]-p[0]) * (q[1]-p[1]));
}

function _keep_left(hull, r) {
    while (hull.length > 1 &&
           turn(hull[hull.length-2], hull[hull.length-1], r) != TURN_LEFT) {
        hull.pop();
    }
    if (!hull.length || hull[hull.length-1] != r) {
        hull.push(r);
    }
    return hull;
}

function _graham_scan(points) {
    points = points.sort()
    const lh = points.reduce(_keep_left, []);
    const uh = points.reverse().reduce(_keep_left, []);
    return lh.concat(uh.slice(1, -1));
}

function _rtangent(hull, p) {
    let [l, r] = [0, hull.length];
    let l_prev = turn(p, hull[0], hull[hull.length-1]);
    let l_next = turn(p, hull[0], hull[(l + 1) % r]);
    while (l < r) {
        const c = (l + r) >> 1;
        const c_prev = turn(p, hull[c],
                            hull[(c - 1 + hull.length) % hull.length]);
        const c_next = turn(p, hull[c],
                            hull[(c + 1 + hull.length) % hull.length]);
        const c_side = turn(p, hull[l], hull[c]);
        if (c_prev !== TURN_RIGHT && c_next !== TURN_RIGHT) {
            return c;
        } else if (c_side === TURN_LEFT &&
                   (l_next === TURN_RIGHT || l_prev === l_next) ||
                   c_side === TURN_RIGHT && c_prev === TURN_RIGHT) {
            r = c;
        } else {
            l = c + 1;
            l_prev = -c_next;
            l_next = turn(p, hull[l], hull[(l + 1) % hull.length]);
        }
    }
    return l;
}

function _min_hull_pt_pair(hulls) {
    let h = 0;
    let p = 0;
    for (let i=0; i<hulls.length; i++) {
        const j = hulls[i].indexOf(Math.min.apply(null, hulls[i]));
        if (hulls[i][j] < hulls[h][p]) {
            [h, p] = [i, j];
        }
    }
    return [h, p];
}

function _dist(p, q) {
    const d0 = p[0] - q[0];
    const d1 = p[1] - q[1];
    return Math.sqrt(d0 * d0 + d1 * d1);
}

function _next_hull_pt_pair(hulls, pair) {
    const p = hulls[pair[0]][pair[1]];

    let next = [pair[0], (pair[1] + 1) % hulls[pair[0]].length];

    for (let h=0; h<hulls.length; h++) {
        if (h === pair[0]) {
            continue;
        }
        const s = _rtangent(hulls[h], p);
        const [q, r] = [hulls[next[0]][next[1]], hulls[h][s]];
        const t = turn(p, q, r);
        if (t === TURN_RIGHT || t === TURN_NONE && _dist(p, r) > _dist(p, q)) {
            next = [h, s];
        }
    }
    return next;
}

function hull(pts) {
    for (let t=0; t<pts.length; t++) {
        const m = 1 << (1 << t);

        const hulls = [];
        for (let i=0; i<pts.length; i+=m) {
            hulls.push(_graham_scan(pts.slice(i, i+m)));
        }
        const hull = [_min_hull_pt_pair(hulls)];
        for (let n=0; n<m; n++) {
            const p = _next_hull_pt_pair(hulls, hull[hull.length-1]);
            if (p[0] === hull[0][0] && p[1] === hull[0][1]) {
                const ret = [];
                for (let [h,i] of hull) {
                    ret.push(hulls[h][i]);
                }
                return ret;
            }
            hull.push(p);
        }
    }
}

function intersect(start0, dir0, start1, dir1) {
    const dd = dir0[0] * dir1[1] - dir0[1] * dir1[0];
    const dx = start1[0] - start0[0];
    const dy = start1[1] - start0[1];
    const t = (dx * dir1[1] - dy * dir1[0]) / dd;
    return [start0[0] + t * dir0[0], start0[1] + t * dir0[1]];
}

export function ombb(pts)
{
    //const convexHull = hull(pts);
    const convexHull = CalcConvexHull(pts);
    const edgeDirs = [];

    for (let i=0; i<convexHull.length; i++) {
        const v1 = convexHull[(i + 1) % convexHull.length];
        const v2 = convexHull[i];

        const v3 = [v1[0]-v2[0], v1[1]-v2[1]];
        const length = Math.sqrt(v3[0]*v3[0] + v3[1]*v3[1]);
        edgeDirs.push([v3[0] / length, v3[1] / length]);
    }

    const minPt = [Number.MAX_VALUE, Number.MAX_VALUE];
    const maxPt = [-Number.MAX_VALUE, -Number.MAX_VALUE];

    let leftIdx;
    let rightIdx;
    let topIdx;
    let bottomIdx;

    for (let i=0; i<convexHull.length; i++) {
        const pt = convexHull[i];

        if (pt[0] < minPt[0]) {
            minPt[0] = pt[0];
            leftIdx = i;
        }

        if (pt[0] > maxPt[0]) {
            maxPt[0] = pt[0];
            rightIdx = i;
        }

        if (pt[1] < minPt[1]) {
            minPt[1] = pt[1];
            bottomIdx = i;
        }

        if (pt[1] > maxPt[1]) {
            maxPt[1] = pt[1];
            topIdx = i;
        }
    }

    let leftDir = [0, -1];
    let rightDir = [0, 1];
    let topDir = [-1, 0];
    let bottomDir = [1, 0];

    function dot(p, q) {
        return p[0]*q[0] + p[1]*q[1];
    }

    let BestObbArea = Number.MAX_VALUE;
    let BestObb = [];

    for (let i=0; i<convexHull.length; i++) {
        const phis = [
            Math.acos(dot(leftDir, edgeDirs[leftIdx])),
            Math.acos(dot(rightDir, edgeDirs[rightIdx])),
            Math.acos(dot(topDir, edgeDirs[topIdx])),
            Math.acos(dot(bottomDir, edgeDirs[bottomIdx]))
        ];

        var lineWithSmallestAngle = phis.indexOf(Math.min.apply(null, phis));
        switch (lineWithSmallestAngle) {
        case 0:
            leftDir = edgeDirs[leftIdx];
            rightDir = [-leftDir[0], -leftDir[1]];
            topDir = [leftDir[1], -leftDir[0]];
            bottomDir = [-topDir[0], -topDir[1]];
            leftIdx = (leftIdx + 1) % convexHull.length;
            break;
        case 1:
            rightDir = edgeDirs[rightIdx];
            leftDir = [-rightDir[0], -rightDir[1]];
            topDir = [leftDir[1], -leftDir[0]];
            bottomDir = [-topDir[0], -topDir[1]];
            rightIdx = (rightIdx + 1) % convexHull.length;
            break;
        case 2:
            topDir = edgeDirs[topIdx];
            bottomDir = [-topDir[0], -topDir[1]];
            leftDir = [bottomDir[1], -bottomDir[0]];
            rightDir = [-leftDir[0], -leftDir[1]];
            topIdx = (topIdx + 1) % convexHull.length;
            break;
        case 3:
            bottomDir = edgeDirs[bottomIdx];
            topDir = [-bottomDir[0], -bottomDir[1]];
            leftDir = [bottomDir[1], -bottomDir[0]];
            rightDir = [-leftDir[0], -leftDir[1]];
            bottomIdx = (bottomIdx + 1) % convexHull.length;
            break;
        }

        const tl = intersect(convexHull[leftIdx], leftDir,
                             convexHull[topIdx], topDir);
        const tr = intersect(convexHull[rightIdx], rightDir,
                             convexHull[topIdx], topDir);
        const bl = intersect(convexHull[bottomIdx], bottomDir,
                             convexHull[leftIdx], leftDir);
        const br = intersect(convexHull[bottomIdx], bottomDir,
                             convexHull[rightIdx], rightDir);

        const dlr = _dist(tl, tr);
        const dtb = _dist(tl, bl);
        const area = dlr * dtb;

        if (area < BestObbArea)
        {
            BestObb = [tl, bl, br, tr];
            BestObbArea = area;
        }
    }

    return BestObb;
}

export function pad_rect(verts, dist) {
    verts = Array.from(verts, v => [v[0], v[1]]);
    for (let [a, b] of [[verts[0], verts[2]], [verts[1], verts[3]]]) {
        let dx = a[0] - b[0];
        let dy = a[1] - b[1];
        if (dx === 0) {
            dy = dist;
        } else if (dy === 0) {
            dx = dist;
        } else {
            const norm = Math.sqrt(dx**2 + dy**2) / dist;
            dx /= norm;
            dy /= norm;
        }
        a[0] += dx;
        b[0] -= dx;
        a[1] += dy;
        b[1] -= dy;
    }
    return verts;
}
