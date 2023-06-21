[Ybus, Yf, Yt] = makeYbus(case57);
Y=full(imag(Ybus));
L=-Y;
for i=1:57
    L(i,i)=0;
    L(i,i)=-sum(L(i,:));
end
[i, j] = find(L);
idx = j >= i; % Find pairs where j > i to exclude permutations
edges_i = i(idx);
edges_j = j(idx);
%disp([edges_i,edges_j])
%disp(size([edges_i,edges_j]))

demand = 26869/10000;
%rng(999)
random_loads = rand(1, 50);
random_loads = random_loads / sum(random_loads) * demand; %normalize so that they sum to 1 * demand
%disp(sum(random_loads))
%disp(random_loads)

generator_indexes = [1, 2, 3, 6, 8, 9, 12];
f_costs = [1,2,3,4,5,6,7]; %cost vector
Pd = zeros(1,57); 
for i=1:57
    if ~ismember(i, generator_indexes)
        Pd(i) = random_loads(1);
        random_loads = random_loads(2:end); %pop first value from random_loads
    end
end
%disp(Pd)

cvx_begin
    variable Pg7(7)
    variable theta(57)
    minimize(dot(Pg7, f_costs))
    subject to
        Pg = cat(1, Pg7(1:3), zeros(2,1), Pg7(4), zeros(1,1), Pg7(5:6), zeros(2,1), Pg7(7), zeros(45,1));
        %disp(Pg)
        %disp([size(Pg), size(Pd)])
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

%disp(theta)    
disp(Pg)
disp(cvx_optval)









%26869/10000
%random_numbers = v * rand(1, 49); = 2.6869

%Pd will be the zeros for generator indexes and the 2.6869 divided by 50
%for the demand indexes, setdiff function might be useful

%part 2:
%dual variable lambda
%...
%lambda: PG_2 - PD_all(:,hour) == L*theta_2


%part 3: (starts with part 1 solution, not related to part 2)
%measurement error added onto p i hat, not p ij hat. thus p ij hat minut
% p ij should be zero
%codes:
%PG_hat(rand_ix) = pghat (randidx) + randi(101, 10000)
%injections idx = randidx

%then in the subject to have to have:
% PG_hat + e == PG_2

%variables to opt over: e(n), edge err (n,n), theta(n)

%for i
%        for j 
%                if sign(L) ~= 0 %there is an edge
%                        pij + edge err ij == abs thetai - theta j * xij







