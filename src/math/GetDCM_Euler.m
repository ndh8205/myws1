function RI2B = GetDCM_Euler( psi, the, phi )
  
    Cpsi = cos ( psi );
    Spsi = sin ( psi );
    Cthe = cos ( the );
    Sthe = sin ( the );
    Cphi = cos ( phi );
    Sphi = sin ( phi );

    RI2B(1,1) =  Cthe * Cpsi ;
    RI2B(1,2) =  Cthe * Spsi ;
    RI2B(1,3) = -Sthe ;
    RI2B(2,1) = -Cphi * Spsi + Sphi * Sthe * Cpsi ;
    RI2B(2,2) =  Cphi * Cpsi + Sphi * Sthe * Spsi ;
    RI2B(2,3) =  Sphi * Cthe ;
    RI2B(3,1) =  Sphi * Spsi + Cphi * Sthe * Cpsi ;
    RI2B(3,2) = -Sphi * Cpsi + Cphi * Sthe * Spsi ;
    RI2B(3,3) =  Cphi * Cthe ;

end