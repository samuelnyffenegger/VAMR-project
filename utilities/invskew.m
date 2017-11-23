function vec = invskew(mat)
    % returns a vector, sd equivalent of matrix multiplication of
    % crossproduct is convertet to its corresponding vector 
    % [vec]_x
    
    %% calculations
    % TODO: check asserts
    %{
    assert(mat(1,2)==-mat(2,1))
    assert(mat(1,3)==-mat(3,1))
    assert(mat(2,3)==-mat(3,2))
    assert(mat(1,1)==0)
    assert(mat(2,2)==0)
    assert(mat(3,3)==0)
    %}
    
    vec = [mat(3,2);mat(1,3);mat(2,1)];
    
end