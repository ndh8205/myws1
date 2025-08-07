%% Limitation Part

function out = LIMIT2( val, lower, upper )

    if( val > upper )
        out = upper ;
    elseif( val < lower )
        out = lower ;
    else
        out = val ;
    end


end