function mat = skew(vec)
    % returns a matrix, sd a crossproduct is calculated using matrix multiplication
    % [vec]_x
    
    %% calculations
    
    mat=[0      -vec(3) vec(2) ; ... 
         vec(3) 0       -vec(1) ; ... 
         -vec(2) vec(1)  0 ];

end