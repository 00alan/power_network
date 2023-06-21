[Ybus, Yf, Yt] = makeYbus(case57);
Y=full(imag(Ybus));
L=-Y;
for i=1:57
    L(i,i)=0;
    L(i,i)=-sum(L(i,:));
end
generator_indexes = [1, 2, 3, 6, 8, 9, 12];
Pg = zeros(1,57);
Pd = zeros(1,57); 
f_costs = [1,2,3,4,5,6,7]; %cost vector
objective_helper_vector = zeros(1,57); %cost vector mult by Pg at generator indexes... dot product this by the diag of L and we get our objective

demand = 26869/10000;
random_loads = rand(1, 50);
random_loads = random_loads / sum(random_loads) * demand; %normalize so that they sum to 1 * demand

for i=1:57
    if ismember(i, generator_indexes)
        Pg(i) = L(i,i);
        objective_helper_vector(i) = f_costs(1);
        f_costs = f_costs(2:end);
    else
        Pd(i) = random_loads(1);
        random_loads = random_loads(2:end); %pop first value from random_loads
    end
end

disp(objective_helper_vector)
disp([size(Pg - Pd), size(L*theta)])
disp([sum(Pg), sum(Pd)])

cvx_begin
    variable theta(57)
    minimize( dot(Pg, objective_helper_vector) )
    subject to
        Pg - Pd == transpose(L*theta);
        theta(4)-theta(6) <= 0;
        theta(4)-theta(6) >= 0;
        theta(8)-theta(9) <= 0;
        theta(8)-theta(9) >= 0;
        for i = 1:56
            for j = i+1:57
                theta(i)-theta(j) <= pi/10;
                theta(j)-theta(i) <= pi/10;
            end
        end
cvx_end

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







