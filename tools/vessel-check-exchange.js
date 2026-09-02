// Heat and mass exchange between vessels -- VESSEL.md §V.10.
//
// The claims under test, each checked against a reference the implementation does not
// share: that the pass's two closed forms really are the exact solutions of the ODEs
// they stand for (so a large step cannot overshoot), that a finite mass transfer is
// the exact integral of the differential one (so `kap ~ m^gamma` is not a small-dm
// approximation), and that a transfer conserves total energy and linear momentum to
// rounding once the mixing dissipation is credited where it belongs.
//
// SI throughout, as the engine is: kg, m, s, Pa, K, J.
const Rs=287.05, gam=1.4, Patm=101325, cv=Rs/(gam-1), cp=gam*Rs/(gam-1);

const ok=(name,err,tol)=>console.log(
  (Math.abs(err)<=tol?'  ok  ':'  FAIL'), name.padEnd(52), err.toExponential(3));

// ---------------------------------------------------------------- 1. heat pair
// C_A T_A' = cond (T_B - T_A),  C_B T_B' = -cond (T_B - T_A).
// The pass solves this in closed form; RK4 at a tiny step is the reference.
console.log('\n1. two-sided heat relaxation is the exact solution, at any step size');
function heatClosed(TA,TB,CA,CB,cond,h){
  const D=(TA-TB)*Math.exp(-cond*(1/CA+1/CB)*h), W=CA*TA+CB*TB;
  return [(W+CB*D)/(CA+CB), (W-CA*D)/(CA+CB)];
}
function heatRK4(TA,TB,CA,CB,cond,h,n){
  const d=y=>[cond*(y[1]-y[0])/CA, -cond*(y[1]-y[0])/CB];
  let y=[TA,TB], dt=h/n;
  for(let i=0;i<n;i++){
    const a=(u,k,s)=>u.map((v,j)=>v+s*k[j]);
    const k1=d(y),k2=d(a(y,k1,dt/2)),k3=d(a(y,k2,dt/2)),k4=d(a(y,k3,dt));
    y=y.map((v,j)=>v+dt/6*(k1[j]+2*k2[j]+2*k3[j]+k4[j]));
  }
  return y;
}
for(const h of [1/120, 1, 60, 1e6]){
  const CA=855, CB=700, cond=64, TA=800, TB=293.15;
  const c=heatClosed(TA,TB,CA,CB,cond,h), r=heatRK4(TA,TB,CA,CB,cond,h,200000);
  ok(`h=${h}: T_A vs RK4`, (c[0]-r[0])/r[0], 1e-11);
  ok(`h=${h}: C_A T_A + C_B T_B conserved`,
     (CA*c[0]+CB*c[1]-(CA*TA+CB*TB))/(CA*TA+CB*TB), 1e-15);
  // the huge-h row is the "cannot overshoot" claim: it lands ON equilibrium, never past
  if(h===1e6) ok('h=1e6: lands exactly on equilibrium', c[0]-c[1], 1e-9);
}

// ---------------------------------------------- 2. discharge is exactly isentropic
// A rigid volume losing mass obeys d(m cv T)/dt = cp T dm/dt, whose exact integral is
// T ~ m^(gamma-1) at fixed V -- i.e. kap = P V^gamma ~ m^gamma. The pass uses that
// closed form for a whole substep's dm at once, so the test that matters is whether
// one big step equals many small ones.
console.log('\n2. removing mass follows the source\'s own isentrope, at any dm');
const V=0.99, m0=2.861, T0=293.15;
const kap0=(m0*Rs*T0/V)*Math.pow(V,gam);
const drop=(kap,m,dm)=>kap*Math.pow((m-dm)/m, gam);
for(const dm of [1e-4, 0.05, 0.5, 1.2]){
  let kapMany=kap0, mMany=m0; const n=20000;
  for(let i=0;i<n;i++){ kapMany=drop(kapMany,mMany,dm/n); mMany-=dm/n; }
  const kapOne=drop(kap0,m0,dm);
  ok(`dm=${dm}: one step vs ${n} steps`, (kapOne-kapMany)/kapMany, 1e-12);
  // and the temperature it implies is the textbook adiabat T ~ rho^(gamma-1)
  const Tone=kapOne*Math.pow(V,1-gam)/((m0-dm)*Rs);
  ok(`dm=${dm}: T/T0 vs (m/m0)^(gamma-1)`,
     Tone/T0 - Math.pow((m0-dm)/m0, gam-1), 1e-12);
}

// -------------------------------------------- 3. a transfer conserves E and p_linear
// Two vessels moving at different velocities, gas crossing between them. The crossing
// mass leaves at the source's own velocity, is merged inelastically into the
// destination, and every joule the merge did not keep as kinetic energy is credited to
// the destination's internal energy. Both books must close exactly.
console.log('\n3. a mass transfer conserves energy and linear momentum exactly');
const mk=(bore,len,mShell,P,T,vx,vy,w,vlen)=>{
  const Vv=bore*len, mass=P*Vv/(Rs*T);
  return {bore,len,mShell,vx,vy,w,vlen, gm:mass, kap:P*Math.pow(Vv,gam)};
};
const tot=v=>{ const M=v.mShell+v.gm, Vv=v.bore*v.len;
  const I=M*(v.bore*v.bore+v.len*v.len)/12, mu=M/12;
  return { M, I, mu,
    ke:0.5*M*(v.vx*v.vx+v.vy*v.vy)+0.5*I*v.w*v.w+0.5*mu*v.vlen*v.vlen,
    U:v.kap*Math.pow(Vv,1-gam)/(gam-1),
    px:M*v.vx, py:M*v.vy };
};
function transfer(src,dst,dm){
  const s0=tot(src), d0=tot(dst), U0s=s0.U, U0d=d0.U;
  src.kap*=Math.pow((src.gm-dm)/src.gm, gam); src.gm-=dm;
  const eCarried=U0s-tot(src).U;
  const keCarried=0.5*dm*(src.vx*src.vx+src.vy*src.vy)
                + 0.5*(dm*(src.bore*src.bore+src.len*src.len)/12)*src.w*src.w
                + 0.5*(dm/12)*src.vlen*src.vlen;
  const svx=src.vx, svy=src.vy;
  dst.gm+=dm; const d1=tot(dst);
  dst.vx=(d0.M*dst.vx+dm*svx)/d1.M; dst.vy=(d0.M*dst.vy+dm*svy)/d1.M;
  dst.w=d0.I*dst.w/d1.I; dst.vlen=d0.mu*dst.vlen/d1.mu;
  const ke1=tot(dst).ke;
  const Ud=U0d+eCarried+keCarried-(ke1-d0.ke);
  dst.kap=Ud*(gam-1)*Math.pow(dst.bore*dst.len, gam-1);
  return {eCarried, keCarried, mixed:eCarried+keCarried-(ke1-d0.ke)-eCarried};
}
{
  const A=mk(0.55,1.8,1500, 2.4*Patm, 293.15,  1.3,-0.7, 2.1, 0.9);
  const B=mk(0.90,0.9, 900,      Patm, 293.15, -0.4, 1.1,-1.4,-0.5);
  const E0=tot(A).ke+tot(A).U+tot(B).ke+tot(B).U;
  const p0=[tot(A).px+tot(B).px, tot(A).py+tot(B).py];
  const M0=A.gm+B.gm;
  const r=transfer(A,B,0.37);
  const E1=tot(A).ke+tot(A).U+tot(B).ke+tot(B).U;
  const p1=[tot(A).px+tot(B).px, tot(A).py+tot(B).py];
  ok('total energy',            (E1-E0)/E0, 1e-14);
  ok('total linear momentum x', (p1[0]-p0[0])/Math.abs(p0[0]), 1e-14);
  ok('total linear momentum y', (p1[1]-p0[1])/Math.abs(p0[1]), 1e-14);
  ok('total gas mass',          (A.gm+B.gm-M0)/M0, 1e-15);
  console.log(`       (the port dissipated ${r.mixed.toFixed(3)} J of organized motion into heat)`);
}

// ---------------------------------------------------- 4. the flow relaxation itself
// dm_A/dt = cond (P_B - P_A) with P = m*s and s = Rs T / V frozen. Same linear form as
// the heat pair; the reference is again RK4 on the raw ODE.
console.log('\n4. two-sided mass relaxation is the exact solution, at any step size');
function flowClosed(mA,mB,sA,sB,cond,h){
  const D0=mA*sA-mB*sB, D1=D0*Math.exp(-cond*(sA+sB)*h);
  const dm=-(D0-D1)/(sA+sB);
  return [mA+dm, mB-dm];
}
function flowRK4(mA,mB,sA,sB,cond,h,n){
  const d=y=>[cond*(y[1]*sB-y[0]*sA), -cond*(y[1]*sB-y[0]*sA)];
  let y=[mA,mB], dt=h/n;
  for(let i=0;i<n;i++){
    const a=(u,k,s)=>u.map((v,j)=>v+s*k[j]);
    const k1=d(y),k2=d(a(y,k1,dt/2)),k3=d(a(y,k2,dt/2)),k4=d(a(y,k3,dt));
    y=y.map((v,j)=>v+dt/6*(k1[j]+2*k2[j]+2*k3[j]+k4[j]));
  }
  return y;
}
for(const h of [1/120, 1, 60, 1e6]){
  const sA=Rs*293.15/0.99, sB=Rs*293.15/0.81, cond=1e-6, mA=2.861, mB=0.9753;
  const c=flowClosed(mA,mB,sA,sB,cond,h), r=flowRK4(mA,mB,sA,sB,cond,h,200000);
  ok(`h=${h}: m_A vs RK4`, (c[0]-r[0])/r[0], 1e-11);
  ok(`h=${h}: total mass conserved`, (c[0]+c[1]-(mA+mB))/(mA+mB), 1e-15);
  if(h===1e6) ok('h=1e6: lands exactly on equal pressure', c[0]*sA-c[1]*sB, 1e-6);
}
console.log('');
