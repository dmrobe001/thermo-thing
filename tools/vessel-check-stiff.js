const a=0.4,gam=1.4,Patm=1.0,mu=0.5;
const kappa=1.0*Math.pow(a*1.0,gam);
const U=L=>kappa*Math.pow(a*L,1-gam)/(gam-1)+Patm*a*L;
const F=L=>kappa*Math.pow(a*L,-gam)*a-Patm*a;
const E=(L,v)=>0.5*mu*v*v+U(L);
const h=1/120;

// Discrete gradient + TRAPEZOIDAL position update.
//   f  chosen so f*(Lnew-L) = -(U(Lnew)-U(L))   [work done == -dU, exactly]
//   vn = v + h f/mu ;  Lnew = L + h(v+vn)/2
// => dKE = (1/2)h f (v+vn) = f*dL = -dU  exactly, for ANY h.
function dgStep(L,v){
  const g=Ln=>{ const dL=Ln-L;
    const f = Math.abs(dL)<1e-14 ? F(L) : -(U(Ln)-U(L))/dL;
    const vn = v + h*f/mu;
    return Ln - L - h*(v+vn)/2; };
  let lo=1e-14, hi=Math.max(L,1)+Math.abs(v)*h*10+1;
  while(g(hi)<0) hi*=2;
  for(let k=0;k<300;k++){ const mid=0.5*(lo+hi); if(g(mid)<0) lo=mid; else hi=mid; }
  const Ln=0.5*(lo+hi), dL=Ln-L;
  const f = Math.abs(dL)<1e-14 ? F(L) : -(U(Ln)-U(L))/dL;
  return [Ln, v+h*f/mu];
}
for(const v0 of [-2,-6,-12,-40,-200]){
  let L=1.0,v=v0; const E0=E(L,v); let minL=9,ok=true;
  for(let i=0;i<200000;i++){ [L,v]=dgStep(L,v);
    if(!(L>0)||!isFinite(L)){ok=false;break;} minL=Math.min(minL,L); }
  console.log({v0, ok, minL:+minL.toExponential(3), relDrift:((E(L,v)-E0)/E0).toExponential(2)});
}
