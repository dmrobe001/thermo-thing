// ============================================================================
//  §14 · SELECTION & INSPECTOR
//  What is selected, and the right-hand panel that reflects and edits it.
//    §14.1  selection state (clearSelection, select*, pickCable)
//    §14.2  renderInspector    (build the panel DOM per selection type)
//    §14.3  updateInspectorLive (per-frame refresh of the live readouts)
// ============================================================================
// ---- §14.1 · selection state ----
let selBody=null, selConstraint=null, selCable=null, selSpring=null, selRotSpring=null, selInteraction=null;
function clearSelection(){ bodies.forEach(b=>b.sel=false); constraints.forEach(c=>c.sel=false); cables.forEach(c=>c.sel=false);
  springs.forEach(s=>s.sel=false); rotSprings.forEach(s=>s.sel=false); interactions.forEach(i=>i.sel=false);
  selBody=null; selConstraint=null; selCable=null; selSpring=null; selRotSpring=null; selInteraction=null;
  renderInspector(); }
function selectBody(i){ clearSelection(); bodies[i].sel=true; selBody=bodies[i]; renderInspector(); }
function selectConstraint(i){ clearSelection(); constraints[i].sel=true; selConstraint=constraints[i]; renderInspector(); }
function selectCable(i){ clearSelection(); cables[i].sel=true; selCable=cables[i]; renderInspector(); }
function selectSpring(i){ clearSelection(); springs[i].sel=true; selSpring=springs[i]; renderInspector(); }
function selectRotSpring(i){ clearSelection(); rotSprings[i].sel=true; selRotSpring=rotSprings[i]; renderInspector(); }
function selectInteraction(i){ clearSelection(); interactions[i].sel=true; selInteraction=interactions[i]; renderInspector(); }
function pickCable(wx,wy){
  for(let i=cables.length-1;i>=0;i--){ if(cableHit(cables[i],wx,wy)) return i; }
  return -1; }
function pickSpring(wx,wy){
  for(let i=springs.length-1;i>=0;i--){ if(springHit(springs[i],wx,wy)) return i; }
  return -1; }
function pickRotSpring(wx,wy){
  for(let i=rotSprings.length-1;i>=0;i--){ if(rotSpringHit(rotSprings[i],wx,wy)) return i; }
  return -1; }

// ---- §14.2b · renderVesselInspector ----
// A vessel's panel. It differs from a body's in exposing a FOURTH coordinate --
// `len`, with its rate `vlen` -- alongside x, y and theta, and in carrying the gas
// state sealed inside it (geometry.js §05.2d).
//
// P, T and the gas mass are three faces of one state at a fixed volume, so editing
// any one has to say what it holds fixed. The rule, shown in the panel itself:
//   temperature -> holds the mass (heating a sealed vessel raises its pressure)
//   pressure    -> holds the temperature (pumping gas in or out)
//   gas mass    -> holds the temperature (the same, stated the other way)
// Each is a deliberate, player-authored change to the gas's energy, exactly as
// typing a velocity into a body's panel is a deliberate change to its kinetic
// energy -- not something the simulation does on its own.
function renderVesselInspector(v){
  const p=document.getElementById('panelBody');
  const P=gasP(v), T=gasT(v), V=vesselVol(v);
  p.innerHTML=`
    <h3>Vessel ${v.id}</h3><p class="sub">gas vessel &middot; fixed bore, variable length</p>
    <div class="card"><div class="cardhead">geometry</div>
      <div class="field"><span class="lab">bore</span><input class="numin" type="number" step="0.02" min="0.02" id="v_bore" value="${v.bore.toFixed(3)}"></div>
      <div class="field"><span class="lab">length</span><input class="numin" type="number" step="0.02" min="0.001" id="v_len" value="${v.len.toFixed(4)}"></div>
      <div class="field"><span class="lab">volume</span><span class="val" id="v_V">${V.toFixed(4)}</span></div>
      <div class="field"><span class="lab">shell mass</span><input class="numin" type="number" step="1" min="0.001" id="v_shell" value="${v.mShell.toFixed(3)}"></div>
      <div class="field"><span class="lab">total mass</span><span class="val" id="v_mass">${v.mass.toFixed(3)}</span></div>
      <div class="field"><span class="lab">inertia</span><span class="val" id="v_I">${v.I.toFixed(4)}</span></div>
      <div class="field"><span class="lab">length inertia</span><span class="val" id="v_mu">${v.mu.toFixed(4)}</span></div>
      <label class="chk"><input type="checkbox" id="v_static" ${v.static?'checked':''}> static (fixed to the world)</label>
      <label class="chk"><input type="checkbox" id="v_lock" ${v.lenLock?'checked':''}> length locked (reservoir)</label>
    </div>
    <div class="card"><div class="cardhead">gas</div>
      <div class="field"><span class="lab">pressure</span><input class="numin" type="number" step="1000" min="0" id="v_P" value="${P.toFixed(1)}"></div>
      <div class="field"><span class="lab">temperature</span><input class="numin" type="number" step="5" min="0.1" id="v_T" value="${T.toFixed(2)}"></div>
      <div class="field"><span class="lab">gas mass</span><input class="numin" type="number" step="0.01" min="0" id="v_gm" value="${v.gas.mass.toFixed(5)}"></div>
      <div class="field"><span class="lab">gamma</span><input class="numin" type="number" step="0.05" min="1.01" id="v_gam" value="${v.gas.gamma.toFixed(3)}"></div>
      <div class="field"><span class="lab">internal energy</span><span class="val" id="v_U">${gasU(v).toFixed(1)}</span></div>
      <div class="field force"><span class="lab">cap force</span><span class="val" id="v_F">${((P-sim.bg.P)*vesselCapArea(v)).toFixed(1)}</span></div>
      <p class="muted" style="margin:8px 0 0">SI throughout: Pa, K, kg, m, J. Ambient is ${(sim.bg.P/1000).toFixed(1)} kPa at ${sim.bg.T.toFixed(2)} K. Editing temperature holds the gas mass; editing pressure or mass holds the temperature. Resizing keeps the gas sealed, so the pressure follows the new volume.</p>
    </div>
    <div class="card"><div class="cardhead">state</div>
      <div class="field"><span class="lab">x</span><input class="numin" type="number" step="0.1" id="v_x" value="${v.x.toFixed(3)}"></div>
      <div class="field"><span class="lab">y</span><input class="numin" type="number" step="0.1" id="v_y" value="${v.y.toFixed(3)}"></div>
      <div class="field"><span class="lab">theta</span><input class="numin" type="number" step="0.05" id="v_th" value="${v.th.toFixed(3)}"></div>
      <div class="field"><span class="lab">vx</span><input class="numin" type="number" step="0.1" id="v_vx" value="${v.vx.toFixed(3)}"></div>
      <div class="field"><span class="lab">vy</span><input class="numin" type="number" step="0.1" id="v_vy" value="${v.vy.toFixed(3)}"></div>
      <div class="field"><span class="lab">w</span><input class="numin" type="number" step="0.1" id="v_w" value="${v.w.toFixed(3)}"></div>
      <div class="field"><span class="lab">len rate</span><input class="numin" type="number" step="0.1" id="v_vlen" value="${v.vlen.toFixed(3)}"></div>
    </div>
    <button class="del" id="v_del">Delete vessel</button>`;
  const num=id=>parseFloat(document.getElementById(id).value);
  const commit=()=>{ renderInspector(); saveState(); };
  // Geometry edits go through resizeVessel, which keeps the gas sealed (mass and
  // temperature carry over) and scales the shell mass with the footprint to hold
  // its density -- the same convention resizeBody uses for an ordinary body.
  const commitGeom=()=>{ const bore=num('v_bore'), len=num('v_len');
    if(isFinite(bore)&&bore>0&&isFinite(len)&&len>0) resizeVessel(v,bore,len);
    projectPositions(8); commit(); };
  document.getElementById('v_bore').onchange=commitGeom;
  document.getElementById('v_len').onchange=commitGeom;
  document.getElementById('v_shell').onchange=()=>{ const m=num('v_shell');
    if(isFinite(m)&&m>0){ v.mShell=m; refreshVessel(v); } commit(); };
  document.getElementById('v_static').onchange=e=>{ setBodyStatic(v,e.target.checked); commit(); };
  document.getElementById('v_lock').onchange=e=>{ v.lenLock=e.target.checked; refreshVessel(v); commit(); };
  document.getElementById('v_P').onchange=()=>{ const Pn=num('v_P');
    if(isFinite(Pn)&&Pn>=0) setVesselGasPT(v,Pn,gasT(v)||sim.bg.T); commit(); };
  document.getElementById('v_T').onchange=()=>{ const Tn=num('v_T');
    if(isFinite(Tn)&&Tn>0) setVesselGasMT(v,v.gas.mass,Tn); commit(); };
  document.getElementById('v_gm').onchange=()=>{ const mn=num('v_gm');
    if(isFinite(mn)&&mn>=0) setVesselGasMT(v,mn,gasT(v)||sim.bg.T); commit(); };
  // gamma changes c_v, hence the internal energy at the same P and V. Hold P and T
  // (the measurable state) and let U follow, rather than the reverse.
  document.getElementById('v_gam').onchange=()=>{ const g=num('v_gam');
    if(isFinite(g)&&g>1.001){ const Pk=gasP(v), Tk=gasT(v)||sim.bg.T; v.gas.gamma=g; setVesselGasPT(v,Pk,Tk); } commit(); };
  const commitPose=()=>{ const x=num('v_x'), y=num('v_y'), th=num('v_th');
    if(isFinite(x)&&isFinite(y)&&isFinite(th)){ v.x=x; v.y=y; v.th=th; projectPositions(8); } commit(); };
  ['v_x','v_y','v_th'].forEach(id=>document.getElementById(id).onchange=commitPose);
  const commitVel=()=>{ const vx=num('v_vx'), vy=num('v_vy'), w=num('v_w'), vl=num('v_vlen');
    if(isFinite(vx)&&isFinite(vy)&&isFinite(w)&&isFinite(vl)){ v.vx=vx; v.vy=vy; v.w=w; v.vlen=vl; } commit(); };
  ['v_vx','v_vy','v_w','v_vlen'].forEach(id=>document.getElementById(id).onchange=commitVel);
  document.getElementById('v_del').onclick=()=>{ const id=v.id;
    constraints=constraints.filter(c=>c.a.id!==id && !(c.b&&c.b.id===id));
    springs=springs.filter(s=>s.a.id!==id && !(s.b&&s.b.id===id));
    rotSprings=rotSprings.filter(s=>s.a.id!==id && s.b.id!==id);
    cables=cables.filter(c=>c.spool.id!==id && c.tether.id!==id);
    dropInteractionsOn(id);
    bodies=bodies.filter(x=>x!==v); clearSelection(); saveState(); };
}

// ---- §14.2 · renderInspector (panel DOM per selection type) ----
// One branch per selection: body, constraint, cable, spring, rotational
// spring, or the empty bench.
function renderInspector(){
  const p=document.getElementById('panelBody');
  if(selBody && selBody.shape==='vessel'){ renderVesselInspector(selBody); return; }
  if(selBody){
    const b=selBody; const isRect=b.shape==='rect';
    p.innerHTML=`
      <h3>Body ${b.id}</h3><p class="sub">${isRect?'rigid rectangle':'rigid disk'}</p>
      <div class="card"><div class="cardhead">properties</div>
        ${isRect
          ? `<div class="field"><span class="lab">width</span><input class="numin" type="number" step="0.05" min="0.16" id="f_rw" value="${(b.hw*2).toFixed(3)}"></div>
             <div class="field"><span class="lab">height</span><input class="numin" type="number" step="0.05" min="0.16" id="f_rh" value="${(b.hh*2).toFixed(3)}"></div>`
          : `<div class="field"><span class="lab">radius</span><input class="numin" type="number" step="0.05" min="0.08" id="f_r" value="${b.r.toFixed(3)}"></div>`}
        <div class="field"><span class="lab">mass</span><input class="numin" type="number" step="0.05" min="0.001" id="f_mass" value="${b.mass.toFixed(3)}"></div>
        <div class="field"><span class="lab">inertia</span><span class="val" id="f_I">${b.I.toFixed(3)}</span></div>
        <label class="chk"><input type="checkbox" id="f_static" ${b.static?'checked':''}> static (fixed to the world)</label>
      </div>
      <div class="card"><div class="cardhead">state</div>
        <div class="field"><span class="lab">x</span><input class="numin" type="number" step="0.1" id="f_x" value="${b.x.toFixed(3)}"></div>
        <div class="field"><span class="lab">y</span><input class="numin" type="number" step="0.1" id="f_y" value="${b.y.toFixed(3)}"></div>
        <div class="field"><span class="lab">theta</span><input class="numin" type="number" step="0.05" id="f_th" value="${b.th.toFixed(3)}"></div>
        <div class="field"><span class="lab">vx</span><input class="numin" type="number" step="0.1" id="f_vx" value="${b.vx.toFixed(3)}"></div>
        <div class="field"><span class="lab">vy</span><input class="numin" type="number" step="0.1" id="f_vy" value="${b.vy.toFixed(3)}"></div>
        <div class="field"><span class="lab">w</span><input class="numin" type="number" step="0.1" id="f_w" value="${b.w.toFixed(3)}"></div>
      </div>
      <button class="del" id="f_del">Delete body</button>`;
    if(isRect){
      const commitSize=()=>{ const w=parseFloat(document.getElementById('f_rw').value), h=parseFloat(document.getElementById('f_rh').value);
        if(isFinite(w)&&w>0.16&&isFinite(h)&&h>0.16) resizeRectAxes(b,w/2,h/2);
        renderInspector(); saveState(); };
      document.getElementById('f_rw').onchange=commitSize;
      document.getElementById('f_rh').onchange=commitSize;
    } else {
      document.getElementById('f_r').onchange=ev=>{ const v=parseFloat(ev.target.value);
        if(isFinite(v)&&v>0.08) resizeBody(b,v);
        renderInspector(); saveState(); };
    }
    document.getElementById('f_mass').onchange=ev=>{ const v=parseFloat(ev.target.value);
      if(isFinite(v)&&v>0.001) setBodyMass(b,v);
      renderInspector(); saveState(); };
    document.getElementById('f_static').onchange=ev=>{ setBodyStatic(b,ev.target.checked); renderInspector(); saveState(); };
    const commitPose=()=>{ const x=parseFloat(document.getElementById('f_x').value),
        y=parseFloat(document.getElementById('f_y').value), th=parseFloat(document.getElementById('f_th').value);
      if(isFinite(x)&&isFinite(y)&&isFinite(th)){ b.x=x; b.y=y; b.th=th; projectPositions(8); }
      renderInspector(); saveState(); };
    document.getElementById('f_x').onchange=commitPose;
    document.getElementById('f_y').onchange=commitPose;
    document.getElementById('f_th').onchange=commitPose;
    const commitVel=()=>{ const vx=parseFloat(document.getElementById('f_vx').value),
        vy=parseFloat(document.getElementById('f_vy').value), w=parseFloat(document.getElementById('f_w').value);
      if(isFinite(vx)&&isFinite(vy)&&isFinite(w)){ b.vx=vx; b.vy=vy; b.w=w; }
      renderInspector(); saveState(); };
    document.getElementById('f_vx').onchange=commitVel;
    document.getElementById('f_vy').onchange=commitVel;
    document.getElementById('f_w').onchange=commitVel;
    document.getElementById('f_del').onclick=()=>{ const id=b.id;
      constraints=constraints.filter(c=>c.a.id!==id && !(c.b&&c.b.id===id));
      springs=springs.filter(s=>s.a.id!==id && !(s.b&&s.b.id===id));
      rotSprings=rotSprings.filter(s=>s.a.id!==id && s.b.id!==id);
      dropInteractionsOn(id);
      bodies=bodies.filter(x=>x!==b); clearSelection(); saveState(); };
  } else if(selConstraint){
    const c=selConstraint;
    const isRod=c.type==='rod', isSlot=c.type==='slot';
    const title = isSlot ? ((c.prismaticA&&c.prismaticB)?'Prismatic slider':'Slot · rail')
                : ({pin:'Pin · hinge',rod:'Rigid rod',
                    belt:'Belt',knife:'Knife-edge wheel',cvt:'Variable gear (CVT)'})[c.type];
    const showTorque = (isRod && (c.weldA||c.weldB)) || (isSlot && (c.prismaticA||c.prismaticB));
    const isBelt=c.type==='belt', isCvt=c.type==='cvt';
    const forceLabel = isBelt?'tension':'|force|';
    let extra='';
    if(isBelt) extra=`<label class="chk"><input type="checkbox" id="f_cross" ${c.sense<0?'checked':''}> crossed belt</label>
        <div class="field"><span class="lab">wrap rA</span><input class="numin" type="number" step="0.02" min="0.02" id="f_rA" value="${c.rA.toFixed(3)}"></div>
        <div class="field"><span class="lab">wrap rB</span><input class="numin" type="number" step="0.02" min="0.02" id="f_rB" value="${c.rB.toFixed(3)}"></div>
        <div class="field"><span class="lab">ratio</span><span class="val" id="f_bratio">${(c.rB/c.rA).toFixed(2)}</span></div>`;
    if(isCvt) extra=`<div class="field"><span class="lab">ratio (d-rA) / rA</span><span class="val" id="f_ratio">--</span></div>`;
    if(isRod) extra=`<label class="chk"><input type="checkbox" id="f_weldA" ${c.weldA?'checked':''}> end A welded${c.a.id==null?' (background)':''}</label>
        <label class="chk"><input type="checkbox" id="f_weldB" ${c.weldB?'checked':''}> end B welded${c.b.id==null?' (background)':''}</label>`;
    if(isSlot) extra=`<label class="chk"><input type="checkbox" id="f_lockA" ${c.prismaticA?'checked':''}> end A prismatic${c.a.id==null?' (background)':''}</label>
        <label class="chk"><input type="checkbox" id="f_lockB" ${c.prismaticB?'checked':''}> end B prismatic${c.b.id==null?' (background)':''}</label>`;
    const note = c.type==='knife' ? 'Nonholonomic: the contact point cannot move sideways, but slides along its heading and pivots freely.'
               : isCvt ? 'Nonholonomic: contact rides A\u2019s rim; the ratio changes as B moves nearer or farther.'
               : isRod ? 'A welded end locks that side\u2019s rotation to the rod; tap an end on the canvas to toggle it, or use the checkboxes here. Reaction is the Lagrange multiplier lambda / h -- run the sim to read it.'
               : isSlot ? 'Two pins is a purely visual guide \u2014 no physical effect. A prismatic end locks its rotation to the rail; once both ends are prismatic the rail also confines position (a rigid prismatic joint). Tap an end on the canvas to toggle it, or use the checkboxes here.'
               : 'Reaction is the Lagrange multiplier lambda / h -- the force this joint carries. Run the sim to read it.';
    p.innerHTML=`
      <h3>${title}</h3><p class="sub">${c.type} constraint</p>
      <div class="card"><div class="cardhead">reaction</div>
        <div class="field force"><span class="lab">${forceLabel}</span><span class="val" id="f_rf">--</span></div>
        ${showTorque?'<div class="field force"><span class="lab">torque</span><span class="val" id="f_rt">--</span></div>':''}
        ${isRod?`<div class="field"><span class="lab">length</span><input class="numin" type="number" step="0.05" min="0.01" id="f_len" value="${c.len.toFixed(3)}"></div>`:''}
        ${extra}
        <p class="muted" style="margin:8px 0 0">${note}</p>
      </div>
      <button class="del" id="f_del">Delete constraint</button>`;
    if(isBelt){
      // recapturing restPhase against the *current* body angles after a wrap-radius
      // edit is the same trick the crossed-belt toggle already uses just below --
      // it keeps the edit from reading as a spurious phase jump next step.
      const recapturePhase=()=>{ const A=bodies[bodyIndex(c.a.id)],B=bodies[bodyIndex(c.b.id)];
        c.restPhase=c.rA*A.th - c.sense*c.rB*B.th; };
      document.getElementById('f_cross').onchange=ev=>{ c.sense=ev.target.checked?-1:1; recapturePhase(); renderInspector(); saveState(); };
      const commitWrap=()=>{ const rA=parseFloat(document.getElementById('f_rA').value), rB=parseFloat(document.getElementById('f_rB').value);
        if(isFinite(rA)&&rA>0.02) c.rA=rA;
        if(isFinite(rB)&&rB>0.02) c.rB=rB;
        recapturePhase(); renderInspector(); saveState(); };
      document.getElementById('f_rA').onchange=commitWrap;
      document.getElementById('f_rB').onchange=commitWrap;
    }
    if(isRod){
      document.getElementById('f_weldA').onchange=ev=>{ setRodWeld(c,'A',ev.target.checked); renderInspector(); saveState(); };
      document.getElementById('f_weldB').onchange=ev=>{ setRodWeld(c,'B',ev.target.checked); renderInspector(); saveState(); };
      document.getElementById('f_len').onchange=ev=>{ const v=parseFloat(ev.target.value);
        if(isFinite(v)&&v>0.01){ c.len=v; projectPositions(8); }
        renderInspector(); saveState(); };
    }
    if(isSlot){
      document.getElementById('f_lockA').onchange=ev=>{ setSlotLock(c,'A',ev.target.checked); renderInspector(); saveState(); };
      document.getElementById('f_lockB').onchange=ev=>{ setSlotLock(c,'B',ev.target.checked); renderInspector(); saveState(); };
    }
    document.getElementById('f_del').onclick=()=>{ constraints=constraints.filter(x=>x!==c); clearSelection(); saveState(); };
  } else if(selInteraction){
    const it=selInteraction, isHeat=it.type==='heat';
    const far = it.vessel.id==null ? 'background' : ('vessel '+it.vessel.id);
    // How many *other* interactions of the same kind sit on this body: a lone one is
    // inert by design, and saying so here is the difference between "nothing is
    // happening" reading as a bug and reading as the rule.
    const partners = interactions.filter(x=>x!==it && x.type===it.type && x.body.id===it.body.id).length;
    p.innerHTML=`
      <h3>${isHeat?'Heat':'Mass-flow'} interaction</h3>
      <p class="sub">body ${it.body.id} &harr; ${far}</p>
      <div class="card"><div class="cardhead">state</div>
        <div class="field"><span class="lab">contact area</span><span class="val" id="i_area">--</span></div>
        <div class="field force"><span class="lab">${isHeat?'heat rate in':'mass rate in'}</span><span class="val" id="i_rate">--</span></div>
        <div class="field"><span class="lab">partners on body</span><span class="val" id="i_pair">${partners}</span></div>
      </div>
      <div class="card"><div class="cardhead">${isHeat?'conductivity':'flow conductance'}</div>
        <div class="field"><span class="lab">k</span><input class="numin" type="number" step="${isHeat?'50':'1e-6'}" min="0" id="i_k" value="${it.k}"></div>
        <p class="muted" style="margin:8px 0 0">${isHeat
          ? 'Heat transfer coefficient, W/(m&sup2;&middot;K). Rate = k_eff &middot; area &middot; (T_far &minus; T_near), solved in closed form over the substep, so it approaches equilibrium exponentially and can never overshoot it at any step size.'
          : 'Flow conductance, kg/(s&middot;m&sup2;&middot;Pa). Rate = k_eff &middot; area &middot; (P_far &minus; P_near) -- the same closed-form relaxation with pressure and mass in place of temperature and capacity. Gas that crosses carries its source&rsquo;s enthalpy, so the emptying side cools along its own isentrope.'}</p>
        <p class="muted" style="margin:8px 0 0">Two interactions of the same kind on the same body are a <em>pair</em>: that body is the wall between what they each name, and the rate uses their k&rsquo;s in series and the smaller of the two contact areas. One on its own moves nothing.</p>
      </div>
      <button class="del" id="i_del">Delete interaction</button>`;
    document.getElementById('i_k').onchange=ev=>{ const v=parseFloat(ev.target.value);
      if(isFinite(v)&&v>=0) it.k=v; renderInspector(); saveState(); };
    document.getElementById('i_del').onclick=()=>{ interactions=interactions.filter(x=>x!==it); clearSelection(); saveState(); };
  } else if(selCable){
    const cb=selCable;
    p.innerHTML=`
      <h3>Cable</h3><p class="sub">tetherball · tension only</p>
      <div class="card"><div class="cardhead">state</div>
        <div class="field force"><span class="lab">tension</span><span class="val" id="cb_T">--</span></div>
        <div class="field"><span class="lab">total length</span><input class="numin" type="number" step="0.1" min="0.01" id="cb_Ltot" value="${cb.Ltot.toFixed(3)}"></div>
        <div class="field"><span class="lab">current length</span><span class="val" id="cb_Lcur">--</span></div>
        <div class="field"><span class="lab">paid out</span><span class="val" id="cb_L">--</span></div>
        <div class="field"><span class="lab">wound turns</span><span class="val" id="cb_W">--</span></div>
        <p class="muted" style="margin:8px 0 0">Fixed total length. Drag the anchor handle to wind/unwind. The spool angle encodes which side the cable winds around and accumulates without bound.</p>
      </div>
      <button class="del" id="cb_del">Delete cable</button>`;
    document.getElementById('cb_Ltot').onchange=ev=>{ const v=parseFloat(ev.target.value);
      if(isFinite(v)&&v>0.01) cb.Ltot=v;
      renderInspector(); saveState(); };
    document.getElementById('cb_del').onclick=()=>{ cables=cables.filter(x=>x!==cb); clearSelection(); saveState(); };
  } else if(selSpring){
    const sp=selSpring;
    p.innerHTML=`
      <h3>Linear spring</h3><p class="sub">force element · F = k(restLen-L)</p>
      <div class="card"><div class="cardhead">state</div>
        <div class="field force"><span class="lab">|force|</span><span class="val" id="sp_F">--</span></div>
        <div class="field"><span class="lab">length</span><span class="val" id="sp_L">--</span></div>
        <div class="field"><span class="lab">rest length</span><input class="numin" type="number" step="0.05" min="0.05" id="sp_rest" value="${sp.restLen.toFixed(3)}"></div>
        <div class="field"><span class="lab">spring constant k</span><input class="numin" type="number" step="0.5" min="0" id="sp_k" value="${sp.k.toFixed(2)}"></div>
        <p class="muted" style="margin:8px 0 0">Hookean force element, not a rigid constraint -- it stores and releases energy rather than being solved exactly. Drag the dashed control point (visible while selected) to set rest length, or type it here.</p>
      </div>
      <button class="del" id="sp_del">Delete spring</button>`;
    document.getElementById('sp_rest').onchange=ev=>{ const v=parseFloat(ev.target.value);
      if(isFinite(v)&&v>0.05) sp.restLen=v;
      renderInspector(); saveState(); };
    document.getElementById('sp_k').onchange=ev=>{ const v=parseFloat(ev.target.value);
      if(isFinite(v)&&v>=0) sp.k=v;
      renderInspector(); saveState(); };
    document.getElementById('sp_del').onclick=()=>{ springs=springs.filter(x=>x!==sp); clearSelection(); saveState(); };
  } else if(selRotSpring){
    const rs=selRotSpring;
    p.innerHTML=`
      <h3>Rotational spring</h3><p class="sub">force element · tau = k(restAngle-relAngle)</p>
      <div class="card"><div class="cardhead">state</div>
        <div class="field force"><span class="lab">|torque|</span><span class="val" id="rs_T">--</span></div>
        <div class="field"><span class="lab">relative angle</span><span class="val" id="rs_ang">--</span></div>
        <div class="field"><span class="lab">rest angle</span><input class="numin" type="number" step="0.05" id="rs_rest" value="${rs.restAngle.toFixed(3)}"></div>
        <div class="field"><span class="lab">spring constant k</span><input class="numin" type="number" step="0.5" min="0" id="rs_k" value="${rs.k.toFixed(2)}"></div>
        <p class="muted" style="margin:8px 0 0">Torsional force element between the two bodies' frame angles (the background reads as a fixed theta=0). Not a rigid constraint.</p>
      </div>
      <button class="del" id="rs_del">Delete rotational spring</button>`;
    document.getElementById('rs_rest').onchange=ev=>{ const v=parseFloat(ev.target.value);
      if(isFinite(v)) rs.restAngle=v;
      renderInspector(); saveState(); };
    document.getElementById('rs_k').onchange=ev=>{ const v=parseFloat(ev.target.value);
      if(isFinite(v)&&v>=0) rs.k=v;
      renderInspector(); saveState(); };
    document.getElementById('rs_del').onclick=()=>{ rotSprings=rotSprings.filter(x=>x!==rs); clearSelection(); saveState(); };
  } else {
    p.innerHTML=`
      <h3>Bench</h3><p class="sub">nothing selected</p>
      <p class="muted">Select a body or constraint to inspect it. Every joint reports the reaction force it carries once the sim is running.</p>
      <div class="card"><div class="cardhead">examples</div>
        <div class="examples">
          <button data-ex="pendulum">Rigid pendulum</button>
          <button data-ex="double">Double pendulum</button>
          <button data-ex="fourbar">Four-bar linkage</button>
          <button data-ex="crank">Slider-crank mechanism</button>
          <button data-ex="skate">Skate (knife-edge)</button>
          <button data-ex="integrator">Wheel integrator (CVT)</button>
          <button data-ex="cable">Cable ratchet</button>
          <button data-ex="gasspring">Gas spring (vessel on ground)</button>
          <button data-ex="spinvessel">Spinning vessel (free)</button>
          <button data-ex="heatpair">Heat exchange (two vessels)</button>
          <button data-ex="flowpair">Gas flow (two vessels)</button>
          <button data-ex="clear">Clear bench</button>
        </div>
      </div>
      <div class="card"><div class="cardhead">controls</div>
        <p class="muted">Wheel to zoom · middle-drag or Alt-drag to pan · Space play/pause · R reset · keys 1-9, b/f/g/h/k/v/c/q tools</p>
      </div>
      ${sceneCardHTML()}`;
    p.querySelectorAll('[data-ex]').forEach(btn=>btn.onclick=()=>loadExample(btn.dataset.ex));
    wireSceneCard();
  }
}
// ---- §14.3 · updateInspectorLive (per-frame readout refresh) ----
// refresh an input's value from live sim state, but never while the user has
// it focused -- clobbering mid-edit would fight their keystrokes
function setLive(id,v){ const el=document.getElementById(id); if(el && document.activeElement!==el) el.value=v; }
function updateInspectorLive(){
  if(selBody && selBody.shape==='vessel' && document.getElementById('v_len')){
    const v=selBody, P=gasP(v), T=gasT(v);
    setLive('v_len',v.len.toFixed(4)); setLive('v_bore',v.bore.toFixed(3));
    setLive('v_x',v.x.toFixed(3)); setLive('v_y',v.y.toFixed(3)); setLive('v_th',v.th.toFixed(3));
    setLive('v_vx',v.vx.toFixed(3)); setLive('v_vy',v.vy.toFixed(3)); setLive('v_w',v.w.toFixed(3));
    setLive('v_vlen',v.vlen.toFixed(3));
    setLive('v_P',P.toFixed(1)); setLive('v_T',T.toFixed(2));
    setLive('v_gm',v.gas.mass.toFixed(5)); setLive('v_shell',v.mShell.toFixed(3));
    document.getElementById('v_V').textContent=vesselVol(v).toFixed(4);
    document.getElementById('v_mass').textContent=v.mass.toFixed(3);
    document.getElementById('v_I').textContent=v.I.toFixed(4);
    document.getElementById('v_mu').textContent=v.mu.toFixed(4);
    document.getElementById('v_U').textContent=gasU(v).toFixed(1);
    document.getElementById('v_F').textContent=((P-sim.bg.P)*vesselCapArea(v)).toFixed(1);
  }
  if(selBody){ const b=selBody;
    if(document.getElementById('f_x')){
      setLive('f_x',b.x.toFixed(3)); setLive('f_y',b.y.toFixed(3)); setLive('f_th',b.th.toFixed(3));
      setLive('f_vx',b.vx.toFixed(3)); setLive('f_vy',b.vy.toFixed(3)); setLive('f_w',b.w.toFixed(3));
      // radius (or width/height)/mass change live while dragging the rim/a
      // corner to resize (§13.6); mass also scales with it there, but is
      // independently editable (setBodyMass)
      if(b.shape==='rect'){ setLive('f_rw',(b.hw*2).toFixed(3)); setLive('f_rh',(b.hh*2).toFixed(3)); }
      else setLive('f_r',b.r.toFixed(3));
      setLive('f_mass',b.mass.toFixed(3));
      document.getElementById('f_I').textContent=b.I.toFixed(3); } }
  if(selConstraint){ const c=selConstraint; const r=reactionOf(c); const el=document.getElementById('f_rf');
    if(el){ if(c.type==='belt') el.textContent=(r?Math.abs(r.val):0).toFixed(2);
      else if(r&&r.fx!==undefined){ el.textContent=Math.hypot(r.fx,r.fy).toFixed(2);
        const t=document.getElementById('f_rt'); if(t&&r.tau!==undefined) t.textContent=r.tau.toFixed(2); } }
    if(c.type==='cvt'){ const A=bodies[bodyIndex(c.a.id)],B=bodies[bodyIndex(c.b.id)];
      const d=Math.hypot(B.x-A.x,B.y-A.y); const er=document.getElementById('f_ratio');
      if(er) er.textContent=((d-A.r)/A.r).toFixed(2); } }
  if(selCable){ const cb=selCable; const f=cableFrame(cb);
    const eT=document.getElementById('cb_T');
    if(eT){ eT.textContent=(cb._lam&&cb._lam.length?Math.hypot(...cb._lam)/sim.h:0).toFixed(2);
      setLive('cb_Ltot',cb.Ltot.toFixed(3));
      document.getElementById('cb_Lcur').textContent=cableCurrentLength(cb,f).toFixed(3);
      document.getElementById('cb_L').textContent=(f?f.paidLength:(cb._Lallow!=null?cb._Lallow:0)).toFixed(3);
      document.getElementById('cb_W').textContent=(f?f.windAngle/(2*Math.PI):0).toFixed(2); } }
  if(selSpring){ const sp=selSpring;
    const [wax,way]=epWorld(sp.a), [wbx,wby]=epWorld(sp.b);
    const L=Math.hypot(wax-wbx,way-wby);
    const eF=document.getElementById('sp_F');
    if(eF){ eF.textContent=Math.abs(sp.k*(sp.restLen-L)).toFixed(3);
      document.getElementById('sp_L').textContent=L.toFixed(3);
      setLive('sp_rest',sp.restLen.toFixed(3)); setLive('sp_k',sp.k.toFixed(2)); } }
  if(selInteraction){ const it=selInteraction;
    const ea=document.getElementById('i_area');
    if(ea){
      const bi=bodyIndex(it.body.id), vi=it.vessel.id!=null?bodyIndex(it.vessel.id):-1;
      // The background has no outline, so its side imposes no area limit -- what the
      // panel shows is this interaction's OWN contact patch, which is the half of the
      // pair's min() the player can actually change by sliding the body.
      const area = (bi>=0 && vi>=0) ? contactArea(bodies[bi],bodies[vi]) : (bi>=0 ? Infinity : 0);
      ea.textContent = isFinite(area) ? area.toFixed(4) : 'unbounded';
      const r=it._rate||0;
      document.getElementById('i_rate').textContent =
        it.type==='heat' ? r.toFixed(2)+' W' : r.toExponential(2)+' kg/s';
      setLive('i_k', it.k);
    } }
  if(selRotSpring){ const rs=selRotSpring;
    const rel=rotSpringRelAngle(rs);
    const eT=document.getElementById('rs_T');
    if(eT){ eT.textContent=Math.abs(rs.k*(rs.restAngle-rel)).toFixed(3);
      document.getElementById('rs_ang').textContent=rel.toFixed(3);
      setLive('rs_rest',rs.restAngle.toFixed(3)); setLive('rs_k',rs.k.toFixed(2)); } }
}
