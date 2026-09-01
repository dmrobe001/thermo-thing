// ============================================================================
//  §05 · GEOMETRY HELPERS
//  Pure helpers with no side effects on the sim: rotation, the body factory,
//  and the world<->screen coordinate maps used by every draw and pick routine.
//    §05.1  rotation & rigid-body point math
//    §05.2  body factory (mass/inertia from radius)
//    §05.3  world<->screen transforms
// ============================================================================

// ---- §05.1 · rotation & rigid-body point math ----
const R = (th,x,y)=>[x*Math.cos(th)-y*Math.sin(th), x*Math.sin(th)+y*Math.cos(th)];
function worldPt(b,off){ const [rx,ry]=R(b.th,off[0],off[1]); return [b.x+rx, b.y+ry, rx, ry]; }
function invMdiag(b){ return b.static?[0,0,0]:[b.invM,b.invM,b.invI]; }

// ---- §05.2 · body factory (mass/inertia from radius) ----
function makeBody(x,y,r,isStatic){
  const mass = Math.PI*r*r;                 // density = 1
  const I = 0.5*mass*r*r;
  return { id:uid++, x, y, th:0, vx:0, vy:0, w:0,
           mass, I, invM:isStatic?0:1/mass, invI:isStatic?0:1/I,
           shape:'circle', r, static:!!isStatic, sel:false, kind:'body' };
}
// Rectangle counterpart of makeBody: hw/hh are the half-width/half-height in
// the body's local (unrotated) frame. Mass is the plate's area (density = 1,
// same convention as the disk); I is the standard uniform-rectangle inertia
// about its own centroid, m*(w^2+h^2)/12 with w=2hw, h=2hh.
function makeRectBody(x,y,hw,hh,isStatic){
  const mass = 4*hw*hh;
  const I = mass*(hw*hw+hh*hh)/3;
  return { id:uid++, x, y, th:0, vx:0, vy:0, w:0,
           mass, I, invM:isStatic?0:1/mass, invI:isStatic?0:1/I,
           shape:'rect', hw, hh, static:!!isStatic, sel:false, kind:'body' };
}
// Recompute I/invM/invI from the body's current mass and shape (uniform
// disk: I = 0.5*mass*r^2; uniform rectangle: I = mass*(hw^2+hh^2)/3). mass
// and the shape dimensions are independent fields -- resizing (§13.6
// resizeBody/resizeRectCorner/resizeRectAxes) scales mass with them to
// preserve density; editing mass directly (inspector.js §14.2 setBodyMass)
// leaves the dimensions untouched. Either path ends here so I/invM/invI
// never drift out of sync with whichever changed.
function refreshInertia(b){
  b.I = b.shape==='rect' ? b.mass*(b.hw*b.hw+b.hh*b.hh)/3 : 0.5*b.mass*b.r*b.r;
  b.invM=b.static?0:1/b.mass; b.invI=b.static?0:1/b.I;
}
function setBodyMass(b,m){ b.mass=m; refreshInertia(b); }

// ---- §05.2b · shape-generic body geometry ----
// A handful of pure functions any picking/snapping/rendering code can call
// without knowing whether `b` is a circle or a rectangle. Everything else in
// the file (rows, forces, inertia) only ever needs mass/I/invM/invI, which
// are already shape-agnostic scalars -- these four are the ones that reach
// down into the actual outline.
//
// Roughly how big the body reads on screen -- used only for the painter sort
// (render.js §11.1) and the rotational-spring spiral's decorative radius
// (constraints.js §06.6), never for physics. A circle's own radius; a
// rectangle's half-diagonal (its farthest point from centre).
function bodyExtentR(b){ return b.shape==='rect' ? Math.hypot(b.hw,b.hh) : b.r; }
// The local-frame point used to draw a body's "theta=0" rim mark (rotational
// spring control points, constraints.js §06.6) -- a circle's rim along its
// own +x, or a rectangle's right-edge midpoint along the same axis.
function bodyRimLocal(b){ return b.shape==='rect' ? [b.hw,0] : [b.r,0]; }
// Is world point (wx,wy) inside b's outline -- the shape-generic core of
// pickBody/pickBodyExcept (tools.js §13.2).
function bodyContains(b,wx,wy){
  if(b.shape==='rect'){
    const [lx,ly]=R(-b.th, wx-b.x, wy-b.y);
    return Math.abs(lx)<=b.hw && Math.abs(ly)<=b.hh;
  }
  const dx=wx-b.x, dy=wy-b.y; return dx*dx+dy*dy<=b.r*b.r;
}
// Nearest point on b's boundary to a world point, and the distance to it --
// the shape-generic core of snapAnchor's edge-snap and (for a circle) the
// resize-rim hit test (tools.js §13.2). For a rectangle: transform to the
// local frame; a point already inside clamps to whichever of the four edges
// is nearest, a point outside clamps to the box (which for a point outside a
// convex box already lands exactly on its boundary).
function bodyEdgePoint(b,wx,wy){
  if(b.shape==='rect'){
    const [lx,ly]=R(-b.th, wx-b.x, wy-b.y);
    let ex,ey;
    if(Math.abs(lx)<=b.hw && Math.abs(ly)<=b.hh){
      const dx=b.hw-Math.abs(lx), dy=b.hh-Math.abs(ly);
      if(dx<dy){ ex=Math.sign(lx||1)*b.hw; ey=ly; } else { ex=lx; ey=Math.sign(ly||1)*b.hh; }
    } else {
      ex=Math.max(-b.hw,Math.min(b.hw,lx)); ey=Math.max(-b.hh,Math.min(b.hh,ly));
    }
    const [wx2,wy2]=R(b.th, ex, ey);
    return [b.x+wx2, b.y+wy2];
  }
  const dx=wx-b.x, dy=wy-b.y, d=Math.hypot(dx,dy)||1e-9;
  return [b.x+dx/d*b.r, b.y+dy/d*b.r];
}
function bodyEdgeDist(b,wx,wy){
  if(b.shape==='rect'){ const [ex,ey]=bodyEdgePoint(b,wx,wy); return Math.hypot(wx-ex,wy-ey); }
  return Math.abs(Math.hypot(wx-b.x,wy-b.y)-b.r);
}

// ---- §05.3 · world<->screen transforms ----
function W(){ return cv.clientWidth; }
function H(){ return cv.clientHeight; }
function w2s(wx,wy){ return [ W()/2 + (wx-cam.x)*cam.scale, H()/2 - (wy-cam.y)*cam.scale ]; }
function s2w(sx,sy){ return [ cam.x + (sx-W()/2)/cam.scale, cam.y - (sy-H()/2)/cam.scale ]; }

// ---- §05.4 · saturating drag pull ----
// Screen-space-capped offset from a world point toward a world target, used
// everywhere a dragged body is pulled toward the cursor (tools.js §13.6 pose
// drag, physics.js §08.1 play-mode grab spring). Grows ~1:1 with the on-screen
// distance for a small, precise tug -- so an unconstrained body still tracks
// the cursor exactly -- but asymptotes to capPx screen pixels' worth of world
// distance however far the cursor is dragged, instead of growing without
// bound. That keeps a constrained body (a rod welded to the background, a
// slider on a fixed rail) from being yanked toward an ever more distant,
// unreachable goal when the drag has a component the joint can't satisfy --
// the joint's own rows stay near-exactly solved instead of being outvoted by
// a runaway pull.
const DRAG_CAP_PX = 80;
function saturatingPull(fromWx, fromWy, toWx, toWy, capPx){
  const dx=toWx-fromWx, dy=toWy-fromWy, d=Math.hypot(dx,dy);
  if(d<1e-9) return [0,0];
  const capW = capPx/cam.scale;
  const mag = capW*(1-Math.exp(-d/capW));
  const s = mag/d;
  return [dx*s, dy*s];
}
