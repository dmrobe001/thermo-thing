// Vessel as a 4-DOF body: q = (cx, cy, th, L)
// T = 1/2 M|cdot|^2 + 1/2 (A + mu L^2) w^2 + 1/2 mu Ldot^2
// V = U(L) + Patm*a*L,   U = kappa*(a L)^(1-g)/(g-1),  P = kappa (aL)^-g
const a=0.4, gam=1.4, Patm=101325;          // bore (m), air, 1 atm
const mShell=800, mGas=0.482;               // kg -- a default-density vessel and its air
const M=mShell+mGas;
// uniform slab in material coords: mu = M/12, A = M a^2/12
const mu=M/12, A=M*a*a/12;
const V0=a*1.0, T0=293.15;                  // L0 = 1 m, 20 C
const Rs=287.05;                            // specific gas constant of air, J/(kg K)
const kappa = (mGas*Rs*T0/V0)*Math.pow(V0,gam);  // P0 * V0^gam

const P_of=L=>kappa*Math.pow(a*L,-gam);
const U_of=L=>kappa*Math.pow(a*L,1-gam)/(gam-1);
const E_of=y=>{const[,, th,L,vx,vy,w,vL]=y;
  return 0.5*M*(vx*vx+vy*vy)+0.5*(A+mu*L*L)*w*w+0.5*mu*vL*vL+U_of(L)+Patm*a*L;};
const Lang_of=y=>(A+mu*y[3]*y[3])*y[6];

function deriv(y){
  const[cx,cy,th,L,vx,vy,w,vL]=y;
  const I=A+mu*L*L;
  // d/dt(I w)=0  ->  I wdot + 2 mu L Ldot w = 0
  const wdot = -2*mu*L*vL*w/I;
  const Ldd  = (mu*L*w*w + (P_of(L)-Patm)*a)/mu;
  return [vx,vy,w,vL, 0,0, wdot, Ldd];
}
function rk4(y,h){
  const ad=(u,k,s)=>u.map((v,i)=>v+s*k[i]);
  const k1=deriv(y),k2=deriv(ad(y,k1,h/2)),k3=deriv(ad(y,k2,h/2)),k4=deriv(ad(y,k3,h));
  return y.map((v,i)=>v+h/6*(k1[i]+2*k2[i]+2*k3[i]+k4[i]));
}
// spinning + breathing, no gravity
let y=[0,0,0,1.0, 0.3,-0.2, 9.0, 0.5];
const E0=E_of(y), L0=Lang_of(y);
let minL=9,maxL=0;
const h=1e-5;
for(let i=0;i<1000000;i++){ y=rk4(y,h); minL=Math.min(minL,y[3]); maxL=Math.max(maxL,y[3]); }
console.log('mu =',mu,' A =',A);
console.log('L range:', minL.toFixed(4), '->', maxL.toFixed(4), ' final L=',y[3].toFixed(4), ' w=',y[6].toFixed(4));
console.log('energy   rel drift:', ((E_of(y)-E0)/E0).toExponential(3));
console.log('ang.mom. rel drift:', ((Lang_of(y)-L0)/L0).toExponential(3));

// --- cross-check: one cap welded down reproduces the classic mu = m/3 ---
// constrain material point f=-1/2 fixed: c_axial = L/2  -> vc = vL/2
// T = 1/2 M (vL/2)^2 + 1/2 mu vL^2 = 1/2 vL^2 (M/4 + M/12) = 1/2 vL^2 (M/3)
console.log('one-end-welded effective mass:', (M/4+mu).toFixed(6), ' vs M/3 =', (M/3).toFixed(6));
