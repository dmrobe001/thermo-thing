// ============================================================================
//  §14 · SELECTION & INSPECTOR
//  What is selected, and the right-hand panel that reflects and edits it.
//    §14.1  selection state (clearSelection, select*, pickGas, pickCable)
//    §14.2  renderInspector    (build the panel DOM per selection type)
//    §14.3  updateInspectorLive (per-frame refresh of the live readouts)
// ============================================================================
// ---- §14.1 · selection state ----
let selBody=null, selConstraint=null, selGas=null, selCable=null, selSpring=null, selRotSpring=null;
let selHeat=null, selFlow=null;
function clearSelection(){ bodies.forEach(b=>b.sel=false); constraints.forEach(c=>c.sel=false); gases.forEach(g=>g.sel=false); cables.forEach(c=>c.sel=false);
  springs.forEach(s=>s.sel=false); rotSprings.forEach(s=>s.sel=false);
  heatInteractions.forEach(h=>h.sel=false); flowInteractions.forEach(f=>f.sel=false);
  selBody=null; selConstraint=null; selGas=null; selCable=null; selSpring=null; selRotSpring=null;
  selHeat=null; selFlow=null; renderInspector(); }
function selectBody(i){ clearSelection(); bodies[i].sel=true; selBody=bodies[i]; renderInspector(); }
function selectConstraint(i){ clearSelection(); constraints[i].sel=true; selConstraint=constraints[i]; renderInspector(); }
function selectGas(i){ clearSelection(); gases[i].sel=true; selGas=gases[i]; renderInspector(); }
function selectCable(i){ clearSelection(); cables[i].sel=true; selCable=cables[i]; renderInspector(); }
function selectSpring(i){ clearSelection(); springs[i].sel=true; selSpring=springs[i]; renderInspector(); }
function selectRotSpring(i){ clearSelection(); rotSprings[i].sel=true; selRotSpring=rotSprings[i]; renderInspector(); }
function selectHeatInteraction(i){ clearSelection(); heatInteractions[i].sel=true; selHeat=heatInteractions[i]; renderInspector(); }
function selectFlowInteraction(i){ clearSelection(); flowInteractions[i].sel=true; selFlow=flowInteractions[i]; renderInspector(); }
function pickGas(wx,wy){
  for(let i=gases.length-1;i>=0;i--){ if(gasHit(gases[i],wx,wy)) return i; }
  return -1; }
function pickHeatInteraction(wx,wy){
  for(let i=heatInteractions.length-1;i>=0;i--){ if(interactionHit(heatInteractions[i],wx,wy)) return i; }
  return -1; }
function pickFlowInteraction(wx,wy){
  for(let i=flowInteractions.length-1;i>=0;i--){ if(interactionHit(flowInteractions[i],wx,wy)) return i; }
  return -1; }
function pickCable(wx,wy){
  for(let i=cables.length-1;i>=0;i--){ if(cableHit(cables[i],wx,wy)) return i; }
  return -1; }
function pickSpring(wx,wy){
  for(let i=springs.length-1;i>=0;i--){ if(springHit(springs[i],wx,wy)) return i; }
  return -1; }
function pickRotSpring(wx,wy){
  for(let i=rotSprings.length-1;i>=0;i--){ if(rotSpringHit(rotSprings[i],wx,wy)) return i; }
  return -1; }

// ---- §14.2 · renderInspector (panel DOM per selection type) ----
// One branch per selection: body, constraint, gas, cable, spring, rotational
// spring, or the empty bench.
function renderInspector(){
  const p=document.getElementById('panelBody');
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
  } else if(selGas){
    const g=selGas; const hasPiston=!!g.piston;
    p.innerHTML=`
      <h3>Gas</h3><p class="sub">${hasPiston?'vessel + piston · ':'fixed vessel · '}P = nT / V</p>
      <div class="card"><div class="cardhead">state</div>
        <div class="field"><span class="lab">pressure P</span><span class="val" id="g_P">--</span></div>
        <div class="field"><span class="lab">volume V</span><span class="val" id="g_V">--</span></div>
        <div class="field"><span class="lab">temp T</span><span class="val" id="g_T">--</span></div>
        <div class="field"><span class="lab">heat+flow Q_dot</span><span class="val" id="g_Q">--</span></div>
      </div>
      <div class="card"><div class="cardhead">gas</div>
        <div class="field"><span class="lab">amount n</span><span class="val" id="g_nL">${g.n.toFixed(2)}</span></div>
        <input type="range" id="g_n" min="0.1" max="10" step="0.1" value="${g.n}" style="width:100%">
        <div class="field"><span class="lab">gamma (index)</span><span class="val" id="g_gL">${g.gamma.toFixed(2)}</span></div>
        <input type="range" id="g_g" min="1.05" max="1.7" step="0.01" value="${g.gamma}" style="width:100%">
        <div class="field"><span class="lab">bore A</span><span class="val" id="g_bL">${g.bore.toFixed(2)}</span></div>
        <input type="range" id="g_b" min="0.3" max="3" step="0.05" value="${g.bore}" style="width:100%">
        ${hasPiston?'':`<div class="field"><span class="lab">length</span><span class="val" id="g_lL">${g.len.toFixed(2)}</span></div>
        <input type="range" id="g_l" min="0.1" max="5" step="0.05" value="${g.len}" style="width:100%">`}
        <p class="muted" style="margin:8px 0 0">${hasPiston
          ?'The movable wall feels internal pressure against the background’s -- add heat/flow interactions (their own tools) to couple this gas to another vessel or the background.'
          :'No movable wall: a fixed-volume vessel, only useful via heat/flow interactions elsewhere.'}</p>
      </div>
      <button class="del" id="g_del">Delete gas</button>`;
    const bind=(id,lab,key,fix)=>{ const el=document.getElementById(id);
      el.oninput=ev=>{ g[key]=parseFloat(ev.target.value); document.getElementById(lab).textContent=g[key].toFixed(fix); saveState(); }; };
    bind('g_n','g_nL','n',2); bind('g_g','g_gL','gamma',2); bind('g_b','g_bL','bore',2);
    if(!hasPiston) bind('g_l','g_lL','len',2);
    document.getElementById('g_del').onclick=()=>{ purgeGas(g); clearSelection(); saveState(); };
  } else if(selHeat || selFlow){
    const it=selHeat||selFlow; const isHeat=!!selHeat;
    const gas = it.gasId!=null ? gases.find(x=>x.id===it.gasId) : null;
    p.innerHTML=`
      <h3>${isHeat?'Heat':'Flow'} interaction</h3>
      <p class="sub">body ${it.bodyId} ↔ ${gas?('gas '+gas.id):'background'}</p>
      <div class="card"><div class="cardhead">state</div>
        <div class="field"><span class="lab">contact area</span><span class="val" id="i_area">--</span></div>
      </div>
      <div class="card"><div class="cardhead">${isHeat?'conductivity':'flow restriction'}</div>
        <div class="field"><span class="lab">k</span><span class="val" id="i_kL">${it.k.toFixed(2)}</span></div>
        <input type="range" id="i_k" min="0" max="20" step="0.1" value="${it.k}" style="width:100%">
        <p class="muted" style="margin:8px 0 0">Two interactions on the same body -- one to each gas/background -- couple those gases through it, at a rate set by their combined k and the smaller of the two contact areas.</p>
      </div>
      <button class="del" id="i_del">Delete interaction</button>`;
    document.getElementById('i_k').oninput=ev=>{ it.k=parseFloat(ev.target.value); document.getElementById('i_kL').textContent=it.k.toFixed(2); saveState(); };
    document.getElementById('i_del').onclick=()=>{
      if(isHeat) heatInteractions=heatInteractions.filter(x=>x!==it); else flowInteractions=flowInteractions.filter(x=>x!==it);
      clearSelection(); saveState(); };
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
      <div class="card"><div class="cardhead">background</div>
        <p class="muted" style="margin:0 0 8px">Counts as an infinite-capacity gas -- any heat/flow interaction pointed at empty space couples to this instead of a placed gas.</p>
        <div class="field"><span class="lab">temp T</span><span class="val" id="bg_TL">${sim.bg.T.toFixed(2)}</span></div>
        <input type="range" id="bg_T" min="0.2" max="3" step="0.05" value="${sim.bg.T}" style="width:100%">
        <div class="field"><span class="lab">pressure P</span><span class="val" id="bg_PL">${sim.bg.P.toFixed(2)}</span></div>
        <input type="range" id="bg_P" min="0" max="5" step="0.05" value="${sim.bg.P}" style="width:100%">
      </div>
      <div class="card"><div class="cardhead">examples</div>
        <div class="examples">
          <button data-ex="pendulum">Rigid pendulum</button>
          <button data-ex="double">Double pendulum</button>
          <button data-ex="fourbar">Four-bar linkage</button>
          <button data-ex="crank">Slider-crank + piston</button>
          <button data-ex="gasspring">Gas spring</button>
          <button data-ex="heatengine">Piston + heat/flow</button>
          <button data-ex="skate">Skate (knife-edge)</button>
          <button data-ex="integrator">Wheel integrator (CVT)</button>
          <button data-ex="cable">Cable ratchet</button>
          <button data-ex="clear">Clear bench</button>
        </div>
      </div>
      <div class="card"><div class="cardhead">controls</div>
        <p class="muted">Wheel to zoom · middle-drag or Alt-drag to pan · Space play/pause · R reset · keys 1-9, b/k/v/c/q/h/f tools</p>
      </div>`;
    p.querySelectorAll('[data-ex]').forEach(btn=>btn.onclick=()=>loadExample(btn.dataset.ex));
    const bindBg=(id,lab,key,fix)=>{ const el=document.getElementById(id);
      el.oninput=ev=>{ sim.bg[key]=parseFloat(ev.target.value); document.getElementById(lab).textContent=sim.bg[key].toFixed(fix); }; };
    bindBg('bg_T','bg_TL','T',2); bindBg('bg_P','bg_PL','P',2);
  }
}
// ---- §14.3 · updateInspectorLive (per-frame readout refresh) ----
// refresh an input's value from live sim state, but never while the user has
// it focused -- clobbering mid-edit would fight their keystrokes
function setLive(id,v){ const el=document.getElementById(id); if(el && document.activeElement!==el) el.value=v; }
function updateInspectorLive(){
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
  if(selGas){ const g=selGas; const f=gasFrame(g);
    // Live from n/T/bore/geometry, not the physics-cached g._P (§08.1 only
    // refreshes it on a substep, so paused -- or an edit before the next
    // substep runs -- would otherwise keep showing the pre-edit pressure).
    const P=g.n*g.T/(g.bore*f.xc);
    const eP=document.getElementById('g_P'); if(eP){ eP.textContent=P.toFixed(3);
      document.getElementById('g_V').textContent=(g.bore*f.xc).toFixed(3);
      document.getElementById('g_T').textContent=g.T.toFixed(3);
      document.getElementById('g_Q').textContent=(g._Q||0).toFixed(3); }
    if(!g.piston) setLive('g_l',g.len); }
  if(selHeat||selFlow){ const it=selHeat||selFlow; const body=bodies[bodyIndex(it.bodyId)];
    const gas = it.gasId!=null ? gases.find(x=>x.id===it.gasId) : null;
    const el=document.getElementById('i_area');
    if(el && body) el.textContent = (gas ? bodyGasOverlapArea(body,gas).toFixed(3) : '∞ (background)'); }
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
  if(selRotSpring){ const rs=selRotSpring;
    const rel=rotSpringRelAngle(rs);
    const eT=document.getElementById('rs_T');
    if(eT){ eT.textContent=Math.abs(rs.k*(rs.restAngle-rel)).toFixed(3);
      document.getElementById('rs_ang').textContent=rel.toFixed(3);
      setLive('rs_rest',rs.restAngle.toFixed(3)); setLive('rs_k',rs.k.toFixed(2)); } }
}
