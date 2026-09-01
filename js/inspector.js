// ============================================================================
//  §14 · SELECTION & INSPECTOR
//  What is selected, and the right-hand panel that reflects and edits it.
//    §14.1  selection state (clearSelection, select*, pickGas, pickCable)
//    §14.2  renderInspector    (build the panel DOM per selection type)
//    §14.3  updateInspectorLive (per-frame refresh of the live readouts)
// ============================================================================
// ---- §14.1 · selection state ----
let selBody=null, selConstraint=null, selGas=null, selCable=null;
function clearSelection(){ bodies.forEach(b=>b.sel=false); constraints.forEach(c=>c.sel=false); gases.forEach(g=>g.sel=false); cables.forEach(c=>c.sel=false);
  selBody=null; selConstraint=null; selGas=null; selCable=null; renderInspector(); }
function selectBody(i){ clearSelection(); bodies[i].sel=true; selBody=bodies[i]; renderInspector(); }
function selectConstraint(i){ clearSelection(); constraints[i].sel=true; selConstraint=constraints[i]; renderInspector(); }
function selectGas(i){ clearSelection(); gases[i].sel=true; selGas=gases[i]; renderInspector(); }
function selectCable(i){ clearSelection(); cables[i].sel=true; selCable=cables[i]; renderInspector(); }
function pickGas(wx,wy){ const tol=12/cam.scale;
  for(let i=gases.length-1;i>=0;i--){ const f=gasFrame(gases[i]);
    if(distSeg(wx,wy,f.hx,f.hy,f.pax,f.pay)<=tol) return i; }
  return -1; }
function pickCable(wx,wy){ const tol=10/cam.scale;
  for(let i=cables.length-1;i>=0;i--){ const f=cableFrame(cables[i]); if(!f)continue;
    if(distSeg(wx,wy,f.T[0],f.T[1],f.Qx,f.Qy)<=tol) return i; }
  return -1; }

// ---- §14.2 · renderInspector (panel DOM per selection type) ----
// One branch per selection: body, constraint, gas, cable, or the empty bench.
function renderInspector(){
  const p=document.getElementById('panelBody');
  if(selBody){
    const b=selBody;
    p.innerHTML=`
      <h3>Body ${b.id}</h3><p class="sub">rigid disk</p>
      <div class="card"><div class="cardhead">properties</div>
        <div class="field"><span class="lab">mass</span><span class="val" id="f_mass">${b.mass.toFixed(3)}</span></div>
        <div class="field"><span class="lab">radius</span><span class="val">${b.r.toFixed(3)}</span></div>
        <div class="field"><span class="lab">inertia</span><span class="val">${b.I.toFixed(3)}</span></div>
      </div>
      <div class="card"><div class="cardhead">state</div>
        <div class="field"><span class="lab">x , y</span><span class="val" id="f_pos"></span></div>
        <div class="field"><span class="lab">θ</span><span class="val" id="f_th"></span></div>
        <div class="field"><span class="lab">speed</span><span class="val" id="f_spd"></span></div>
      </div>
      <button class="del" id="f_del">Delete body</button>`;
    document.getElementById('f_del').onclick=()=>{ const id=b.id;
      constraints=constraints.filter(c=>c.a.id!==id && !(c.b&&c.b.id===id));
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
        <div class="field"><span class="lab">ratio</span><span class="val">${(c.rB/c.rA).toFixed(2)}</span></div>`;
    if(isCvt) extra=`<div class="field"><span class="lab">ratio (d−rA) ⁄ rA</span><span class="val" id="f_ratio">—</span></div>`;
    if(isRod) extra=`<label class="chk"><input type="checkbox" id="f_weldA" ${c.weldA?'checked':''}> end A welded${c.a.id==null?' (background)':''}</label>
        <label class="chk"><input type="checkbox" id="f_weldB" ${c.weldB?'checked':''}> end B welded${c.b.id==null?' (background)':''}</label>`;
    if(isSlot) extra=`<label class="chk"><input type="checkbox" id="f_lockA" ${c.prismaticA?'checked':''}> end A prismatic${c.a.id==null?' (background)':''}</label>
        <label class="chk"><input type="checkbox" id="f_lockB" ${c.prismaticB?'checked':''}> end B prismatic${c.b.id==null?' (background)':''}</label>`;
    const note = c.type==='knife' ? 'Nonholonomic: the contact point cannot move sideways, but slides along its heading and pivots freely.'
               : isCvt ? 'Nonholonomic: contact rides A\u2019s rim; the ratio changes as B moves nearer or farther.'
               : isRod ? 'A welded end locks that side\u2019s rotation to the rod; tap an end on the canvas to toggle it, or use the checkboxes here. Reaction is the Lagrange multiplier λ ⁄ h — run the sim to read it.'
               : isSlot ? 'Two pins is a purely visual guide \u2014 no physical effect. A prismatic end locks its rotation to the rail; once both ends are prismatic the rail also confines position (a rigid prismatic joint). Tap an end on the canvas to toggle it, or use the checkboxes here.'
               : 'Reaction is the Lagrange multiplier λ ⁄ h — the force this joint carries. Run the sim to read it.';
    p.innerHTML=`
      <h3>${title}</h3><p class="sub">${c.type} constraint</p>
      <div class="card"><div class="cardhead">reaction</div>
        <div class="field force"><span class="lab">${forceLabel}</span><span class="val" id="f_rf">—</span></div>
        ${showTorque?'<div class="field force"><span class="lab">torque</span><span class="val" id="f_rt">—</span></div>':''}
        ${c.type==='rod'?`<div class="field"><span class="lab">length</span><span class="val">${c.len.toFixed(3)}</span></div>`:''}
        ${extra}
        <p class="muted" style="margin:8px 0 0">${note}</p>
      </div>
      <button class="del" id="f_del">Delete constraint</button>`;
    if(isBelt){ document.getElementById('f_cross').onchange=ev=>{ const A=bodies[bodyIndex(c.a.id)],B=bodies[bodyIndex(c.b.id)];
      c.sense=ev.target.checked?-1:1; c.restPhase=c.rA*A.th - c.sense*c.rB*B.th; renderInspector(); saveState(); }; }
    if(isRod){
      document.getElementById('f_weldA').onchange=ev=>{ setRodWeld(c,'A',ev.target.checked); renderInspector(); saveState(); };
      document.getElementById('f_weldB').onchange=ev=>{ setRodWeld(c,'B',ev.target.checked); renderInspector(); saveState(); };
    }
    if(isSlot){
      document.getElementById('f_lockA').onchange=ev=>{ setSlotLock(c,'A',ev.target.checked); renderInspector(); saveState(); };
      document.getElementById('f_lockB').onchange=ev=>{ setSlotLock(c,'B',ev.target.checked); renderInspector(); saveState(); };
    }
    document.getElementById('f_del').onclick=()=>{ constraints=constraints.filter(x=>x!==c); clearSelection(); saveState(); };
  } else if(selGas){
    const g=selGas;
    p.innerHTML=`
      <h3>Gas piston</h3><p class="sub">force element · P = nRT ⁄ V</p>
      <div class="card"><div class="cardhead">state</div>
        <div class="field"><span class="lab">pressure P</span><span class="val" id="g_P">—</span></div>
        <div class="field"><span class="lab">volume V</span><span class="val" id="g_V">—</span></div>
        <div class="field"><span class="lab">temp T</span><span class="val" id="g_T">—</span></div>
        <div class="field"><span class="lab">heat Q̇</span><span class="val" id="g_Q">—</span></div>
      </div>
      <div class="card"><div class="cardhead">gas</div>
        <div class="field"><span class="lab">amount n</span><span class="val" id="g_nL">${g.n.toFixed(1)}</span></div>
        <input type="range" id="g_n" min="0.5" max="10" step="0.1" value="${g.n}" style="width:100%">
        <div class="field"><span class="lab">γ (index)</span><span class="val" id="g_gL">${g.gamma.toFixed(2)}</span></div>
        <input type="range" id="g_g" min="1.05" max="1.7" step="0.01" value="${g.gamma}" style="width:100%">
        <div class="field"><span class="lab">bore A</span><span class="val" id="g_bL">${g.bore.toFixed(2)}</span></div>
        <input type="range" id="g_b" min="0.3" max="3" step="0.05" value="${g.bore}" style="width:100%">
      </div>
      <div class="card"><div class="cardhead">reservoir</div>
        <label class="chk"><input type="checkbox" id="g_conn" ${g.connected?'checked':''}> connected</label>
        <div class="field"><span class="lab">T_res</span><span class="val" id="g_TrL">${g.Tres.toFixed(2)}</span></div>
        <input type="range" id="g_Tr" min="0.2" max="3" step="0.05" value="${g.Tres}" style="width:100%">
        <div class="field"><span class="lab">conductance κ</span><span class="val" id="g_kL">${g.kappa.toFixed(1)}</span></div>
        <input type="range" id="g_k" min="0" max="30" step="0.5" value="${g.kappa}" style="width:100%">
        <p class="muted" style="margin:8px 0 0">κ = 0 is adiabatic (a gas spring). Connect a warm/cold reservoir to move heat across the boundary at finite rate.</p>
      </div>
      <button class="del" id="g_del">Delete gas</button>`;
    const bind=(id,lab,key,fix)=>{ const el=document.getElementById(id);
      el.oninput=ev=>{ g[key]=parseFloat(ev.target.value); document.getElementById(lab).textContent=g[key].toFixed(fix); saveState(); }; };
    bind('g_n','g_nL','n',1); bind('g_g','g_gL','gamma',2); bind('g_b','g_bL','bore',2);
    bind('g_Tr','g_TrL','Tres',2); bind('g_k','g_kL','kappa',1);
    document.getElementById('g_conn').onchange=ev=>{ g.connected=ev.target.checked; saveState(); };
    document.getElementById('g_del').onclick=()=>{ gases=gases.filter(x=>x!==g); clearSelection(); saveState(); };
  } else if(selCable){
    const cb=selCable;
    p.innerHTML=`
      <h3>Cable</h3><p class="sub">tetherball · tension only</p>
      <div class="card"><div class="cardhead">state</div>
        <div class="field force"><span class="lab">tension</span><span class="val" id="cb_T">—</span></div>
        <div class="field"><span class="lab">total length</span><span class="val" id="cb_Ltot">${cb.Ltot.toFixed(3)}</span></div>
        <div class="field"><span class="lab">current length</span><span class="val" id="cb_Lcur">—</span></div>
        <div class="field"><span class="lab">paid out</span><span class="val" id="cb_L">—</span></div>
        <div class="field"><span class="lab">wound turns</span><span class="val" id="cb_W">—</span></div>
        <p class="muted" style="margin:8px 0 0">Fixed total length. Drag the anchor handle to wind/unwind. The spool angle encodes which side the cable winds around and accumulates without bound.</p>
      </div>
      <button class="del" id="cb_del">Delete cable</button>`;
    document.getElementById('cb_del').onclick=()=>{ cables=cables.filter(x=>x!==cb); clearSelection(); saveState(); };
  } else {
    p.innerHTML=`
      <h3>Bench</h3><p class="sub">nothing selected</p>
      <p class="muted">Select a body or constraint to inspect it. Every joint reports the reaction force it carries once the sim is running.</p>
      <div class="card"><div class="cardhead">examples</div>
        <div class="examples">
          <button data-ex="pendulum">Rigid pendulum</button>
          <button data-ex="double">Double pendulum</button>
          <button data-ex="fourbar">Four-bar linkage</button>
          <button data-ex="crank">Slider-crank + piston</button>
          <button data-ex="gasspring">Gas spring</button>
          <button data-ex="skate">Skate (knife-edge)</button>
          <button data-ex="integrator">Wheel integrator (CVT)</button>
          <button data-ex="cable">Cable ratchet</button>
          <button data-ex="clear">Clear bench</button>
        </div>
      </div>
      <div class="card"><div class="cardhead">controls</div>
        <p class="muted">Wheel to zoom · middle-drag or Alt-drag to pan · Space play/pause · R reset · keys 1–7, b/k/v/c tools</p>
      </div>`;
    p.querySelectorAll('[data-ex]').forEach(btn=>btn.onclick=()=>loadExample(btn.dataset.ex));
  }
}
// ---- §14.3 · updateInspectorLive (per-frame readout refresh) ----
function updateInspectorLive(){
  if(selBody){ const b=selBody;
    const pos=document.getElementById('f_pos'); if(pos){ pos.textContent=`${b.x.toFixed(2)}, ${b.y.toFixed(2)}`;
      document.getElementById('f_th').textContent=b.th.toFixed(2);
      document.getElementById('f_spd').textContent=Math.hypot(b.vx,b.vy).toFixed(2); } }
  if(selConstraint){ const c=selConstraint; const r=reactionOf(c); const el=document.getElementById('f_rf');
    if(el){ if(c.type==='belt') el.textContent=(r?Math.abs(r.val):0).toFixed(2);
      else if(r&&r.fx!==undefined){ el.textContent=Math.hypot(r.fx,r.fy).toFixed(2);
        const t=document.getElementById('f_rt'); if(t&&r.tau!==undefined) t.textContent=r.tau.toFixed(2); } }
    if(c.type==='cvt'){ const A=bodies[bodyIndex(c.a.id)],B=bodies[bodyIndex(c.b.id)];
      const d=Math.hypot(B.x-A.x,B.y-A.y); const er=document.getElementById('f_ratio');
      if(er) er.textContent=((d-A.r)/A.r).toFixed(2); } }
  if(selGas){ const g=selGas; const f=gasFrame(g); const P=g._P||g.n*g.T/(g.bore*f.xc);
    const eP=document.getElementById('g_P'); if(eP){ eP.textContent=P.toFixed(3);
      document.getElementById('g_V').textContent=(g.bore*f.xc).toFixed(3);
      document.getElementById('g_T').textContent=g.T.toFixed(3);
      document.getElementById('g_Q').textContent=(g._Q||0).toFixed(3); } }
  if(selCable){ const cb=selCable; const f=cableFrame(cb);
    const eT=document.getElementById('cb_T');
    if(eT){ eT.textContent=(cb._lam&&cb._lam.length?Math.hypot(...cb._lam)/sim.h:0).toFixed(2);
      document.getElementById('cb_Ltot').textContent=cb.Ltot.toFixed(3);
      document.getElementById('cb_Lcur').textContent=cableCurrentLength(cb,f).toFixed(3);
      document.getElementById('cb_L').textContent=(f?f.paidLength:(cb._Lallow!=null?cb._Lallow:0)).toFixed(3);
      document.getElementById('cb_W').textContent=(f?f.windAngle/(2*Math.PI):0).toFixed(2); } }
}
