[Ybus, Yf, Yt] = makeYbus(case57);
Y=full(imag(Ybus));
L=-Y;
for i=1:57
    L(i,i)=0;
    L(i,i)=-sum(L(i,:));
end
[i, j] = find(L);
idx = j >= i; % Find pairs where j > i to exclude permutations
pairs = [i(idx), j(idx)];

demand = 26869/10000;
random_loads = rand(1, 50);
random_loads = random_loads / sum(random_loads) * demand; %normalize so that they sum to 1 * demand
generator_indexes = [1, 2, 3, 6, 8, 9, 12];
f_costs = [1,2,3,4,5,6,7]; %cost vector
Pd = zeros(1,57); 
for i=1:57
    if ~ismember(i, generator_indexes)
        Pd(i) = random_loads(1);
        random_loads = random_loads(2:end); %pop first value from random_loads
    end
end

cvx_begin
    variable Pg7(7)
    variable theta(57)
    minimize(dot(Pg7, f_costs))
    subject to
        Pg = cat(1, Pg7(1:3), zeros(2,1), Pg7(4), zeros(1,1), Pg7(5:6), zeros(2,1), Pg7(7), zeros(45,1));
        Pg - transpose(Pd) == L*theta;
        theta(4)-theta(6) == 0;
        theta(8)-theta(9) == 0;
        for row = 1:size(pairs, 1)
            pair = pairs(row, :);
            i = pair(1);
            j = pair(2);
            theta(i)-theta(j) <= pi/10;
            theta(j)-theta(i) <= pi/10;
        end
cvx_end
Pg
%transpose(Pd)
Pg7
cvx_optval