// Gas uniform between head (material f=0) and cap (f=1), separation x.
// Exact KE in coords (v_head, xdot):  1/2*[m, m/2; m/2, m/3]
// Old design: cap body carries mass m/3, head carries none of the gas.
// New design: one body, centered coords (cdot, Ldot), KE = 1/2 m cdot^2 + 1/2 (m/12) Ldot^2
const m=1;
const KEexact=(vh,xd)=>0.5*m*vh*vh + 0.5*m*vh*xd + 0.5*(m/3)*xd*xd;
const KEold  =(vh,xd)=>0.5*(m/3)*(vh+xd)*(vh+xd);            // cap mass m/3 only
const KEnew  =(vh,xd)=>{const cd=vh+xd/2; return 0.5*m*cd*cd + 0.5*(m/12)*xd*xd;};
console.log('  v_head  xdot |   exact      old(m/3 cap)   new(centered)');
for(const [vh,xd] of [[0,1],[1,0],[1,1],[2,-3],[0.7,0.3]])
  console.log(`  ${vh.toString().padStart(5)} ${xd.toString().padStart(5)} | `+
    [KEexact(vh,xd),KEold(vh,xd),KEnew(vh,xd)].map(v=>v.toFixed(6).padStart(12)).join(' '));
