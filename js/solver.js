// ============================================================================
//  §07 · LINEAR SOLVE
//  Dense Gauss-Jordan with partial pivoting, solving one symmetric system
//  K·x = b in place. K here is the Schur complement J·M^-1·J^T (assembled in
//  §08.3 and §09), i.e. the reduced form of the KKT system in spec §3.2 -- not
//  the full saddle matrix, and not (yet) factored with Cholesky or split per
//  island. See the design-drift notes in the spec for where this sits on the
//  solver ladder (spec §3.5, §3.6).
// ============================================================================
// ---- §07.1 · solveLinear ----
function solveLinear(Kt, b, n){
  for(let i=0;i<n;i++){
    let p=i, mx=Math.abs(Kt[i][i]);
    for(let r=i+1;r<n;r++){ const v=Math.abs(Kt[r][i]); if(v>mx){mx=v;p=r;} }
    if(mx<1e-12) continue;                 // regularization normally prevents this
    if(p!==i){ const t=Kt[p];Kt[p]=Kt[i];Kt[i]=t; const tb=b[p];b[p]=b[i];b[i]=tb; }
    const piv=Kt[i][i];
    for(let r=0;r<n;r++){ if(r===i)continue; const f=Kt[r][i]/piv;
      if(f){ for(let c=i;c<n;c++) Kt[r][c]-=f*Kt[i][c]; b[r]-=f*b[i]; } }
  }
  const x=new Array(n).fill(0);
  for(let i=0;i<n;i++) x[i]=Math.abs(Kt[i][i])>1e-12 ? b[i]/Kt[i][i] : 0;
  return x;
}
