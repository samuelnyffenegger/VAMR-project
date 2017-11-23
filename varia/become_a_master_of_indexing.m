% https://ch.mathworks.com/company/newsletters/articles/matrix-indexing-in-matlab.html

clear all; close all; clc;

%% Indexing Vectors
clc
v = [16 5 9 4 2 11 7 14];
v(3) % Extract the third elemen
v([1 5 6]) % Extract the first, fifth, and sixth elements
v(3:7) % Extract the third through the seventh elements
v2 = v([5:8 1:4])     % Extract and swap the halves of v
v(end) % Extract the last element
v(5:end) % Extract the fifth through the last elements   
v(2:end-1) % Extract the second through the next-to-last elements
v(1:2:end) % Extract all the odd elements
v(end:-1:1) % Reverse the order of elements
v([2 3 4]) = [10 15 20] % Replace some elements of v
v([2 3]) = 30  % Replace second and third elements by 30

%% Indexing Matrices with Two Subscripts
clc
A = magic(4)
A(2,4) % Extract the element in row 2, column 4
A(2:4,1:2)
A(3,:)   % Extract third row
A(:,end)   % Extract last column

%% Linear Indexing
A(14)
A([6 12 15])
A([2 7 16])
idx = sub2ind(size(A), [2 3 4], [1 2 4])
A(idx)

%% Advanced Examples Using Linear Indexing
% tba

%% Logical Indexing
clc
A(A > 12)
[Y,I] = max(A, [], 2);
B = zeros(size(A));
B(sub2ind(size(A), 1:length(I), I')) = Y;
B(isnan(B)) = 0
str = 'Hello World'
str(isspace(str)) = '_'

A(3,2) = NaN;
nan_locations = find(isnan(A));
A(nan_locations) = 0;
A = filter2(ones(3,3), A);
A(nan_locations) = NaN;


