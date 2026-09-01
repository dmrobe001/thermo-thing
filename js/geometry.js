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
           r, static:!!isStatic, sel:false, kind:'body' };
}
// Recompute I/invM/invI from the body's current mass and radius (uniform
// disk: I = 0.5*mass*r^2). mass and r are independent fields -- resizing
// (§13.6 resizeBody) scales mass with them to preserve density; editing mass
// directly (inspector.js §14.2 setBodyMass) leaves r untouched. Either path
// ends here so I/invM/invI never drift out of sync with whichever changed.
function refreshInertia(b){
  b.I=0.5*b.mass*b.r*b.r;
  b.invM=b.static?0:1/b.mass; b.invI=b.static?0:1/b.I;
}
function setBodyMass(b,m){ b.mass=m; refreshInertia(b); }

// ---- §05.3 · world<->screen transforms ----
function W(){ return cv.clientWidth; }
function H(){ return cv.clientHeight; }
function w2s(wx,wy){ return [ W()/2 + (wx-cam.x)*cam.scale, H()/2 - (wy-cam.y)*cam.scale ]; }
function s2w(sx,sy){ return [ cam.x + (sx-W()/2)/cam.scale, cam.y - (sy-H()/2)/cam.scale ]; }
