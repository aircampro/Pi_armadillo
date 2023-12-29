% Define a function make_list(X, N) that generates a list with N elements X, and a function tabulate(F, N, M) 
% that stores and returns the result of applying function F to values from integers N to M in a list
% ==========================================================================================================
make_list(_, 0) -> [];
make_list(X, N) when N > 0 -> [X | make_list(X, N - 1)].
tabulate(_, N, M) when N > M -> [];
tabulate(F, N, M) -> [F(N) | tabulate(F, N + 1, M)].

> yaep:make_list(a, 10).
[a,a,a,a,a,a,a,a,a,a]
> yaep:make_list("hello", 5).
["hello","hello","hello","hello","hello"]

> yaep:tabulate(fun(X) -> X end, 1, 10).
[1,2,3,4,5,6,7,8,9,10]

> yaep:tabulate(fun(X) -> X * X end, 1, 10).
[1,4,9,16,25,36,49,64,81,100]

> yaep:tabulate(fun(X) -> X * 1 end, 1, 5).
[2,3,4,5,6]

make_listi(_, 0, A) -> A;
make_listi(X, N, A) -> make_listi(X, N - 1, [X | A]).
make_listi(X, N) when N > 0 -> make_listi(X, N, []).

tabulatei(F, N, N, A) -> [F(N) | A];
tabulatei(F, N, M, A) -> tabulatei(F, N, M - 1, [F(M) | A]).
tabulatei(F, N, M) when N < M -> tabulatei(F, N, M, []).

% Define a function remove(X, Xs) that removes all elements X from the list Xs, and a 
% function remove_if(Pred, Xs) that removes all elements for which the predicate Pred returns true.
% ==========================================================================================================
remove(X, Xs) -> [Y || Y <- Xs, X =/= Y].
remove_if(Pred, Xs) -> [X || X <- Xs, not Pred(X)].

> yaep:remove(a, [a, b, c, a, b, c, a]).
[b,c,b,c]
> yaep:remove_if(fun(X) -> X rem 2 =:= 0 end, [1, 2, 3, 4, 5, 6, 7, 8]).
[1,3,5,7]
> yaep:remove_if(fun(X) -> X rem 4 =:= 0 end, [1, 2, 3, 4, 5, 6, 7, 8]).
[1, 2, 3, 5, 6, 7]

% Define a function flatten(Xs) that flattens the list Xs.
% ==========================================================================================================
flatten([]) -> [];
flatten([X | Xs]) -> flatten(X) ++ flatten(Xs);
flatten(X) -> [X].

flatten1([], A) -> A;
flatten1([X | Xs], A) -> flatten1(X, flatten1(Xs, A));
flatten1(X, A) -> [X | A].

> yaep:flatten([a, b, [c, d, [e | f], g], h]).
[a,b,c,d,e,f,g,h]
> yaep:flatten([a, b, [c, d, [e, [] | f], g], h]).
[a,b,c,d,e,f,g,h]

% Define a function concat(Xs) that concatenates multiple lists stored in list Xs
% ==========================================================================================================
concat([]) -> [];
concat([X | Xs]) -> X ++ concat(Xs).

> yaep:concat([[a, b, c], [d, e], [f, g, h, i]]).
[a,b,c,d,e,f,g,h,i]

% Apply the function F to the list stored in list Xs and define the function flatmap(F, Xs) to concatenate the result
% ==========================================================================================================
flatmap(, []) -> [];
flatmap(F, [X | Xs]) -> F(X) ++ flatmap(F, Xs).

> yaep:flatmap(fun(X) -> [a | X] end, [[b, c], [d, e], [f, g], [h, i]]).
[a,b,c,a,d,e,a,f,g,a,h,i]

% Define a permutation(N, Xs) function to find the permutation that selects N elements from the list Xs. 
% Note that the generated permutations are stored in a list and returned.
% ==========================================================================================================
permutation(0, _) -> [[]];
permutation(N, Xs) when N > 0 ->
  flatmap(fun(X) -> lists:map(fun(Y) -> [X | Y] end,
                              permutation(N - 1, lists:delete(X, Xs))) end,
          Xs).

% 高階関数版
permutation(F, 0, _, A) -> F(lists:reverse(A));
permutation(F, N, Xs, A) ->
  lists:foreach(fun(X) -> permutation(F, N - 1, lists:delete(X, Xs), [X | A]) end, Xs).
permutation(F, N, Xs) when N > 0 -> permutation(F, N, Xs, []).

> yaep:permutation(3, [a, b, c]).
[[a,b,c],[a,c,b],[b,a,c],[b,c,a],[c,a,b],[c,b,a]]

% Define a function repeat_perm(N, Xs) to find a permutation that allows duplicates from the list Xs and selects N elements. 
% Note that the generated permutations are stored in a list and returned.
% ==========================================================================================================
repeat_perm(0, _) -> [[]];
repeat_perm(N, Xs) when N > 0 ->
  flatmap(fun(X) -> lists:map(fun(Y) -> [X | Y] end,
                              repeat_perm(N - 1, Xs)) end,
          Xs).

% 高階関数版
repeat_perm(F, 0, _, A) -> F(lists:reverse(A));
repeat_perm(F, N, Xs, A) ->
  lists:foreach(fun(X) -> repeat_perm(F, N - 1, Xs, [X | A]) end, Xs).
repeat_perm(F, N, Xs) when N > 0 -> repeat_perm(F, N, Xs, []).

> yaep:repeat_perm(2, [a, b, c]).
[[a,a],[a,b],[a,c],[b,a],[b,b],[b,c],[c,a],[c,b],[c,c]]

% Define a function combination(N, Xs) that finds a combination that selects N elements 
% from the list Xs. Note that the generated combinations are stored in a list and returned.
% ==========================================================================================================
combination(0, _) -> [[]];
combination(N, Xs) when length(Xs) =:= N -> [Xs];
combination(N, [Y | Ys]) ->
  lists:map(fun(X) -> [Y | X] end, combination(N - 1, Ys)) ++ combination(N, Ys).

% 高階関数版
combination(F, 0, _, A) -> F(lists:reverse(A));
combination(F, N, Xs, A) when length(Xs) =:= N -> F(lists:reverse(A) ++ Xs);
combination(F, N, [X | Xs], A) ->
  combination(F, N - 1, Xs, [X | A]), combination(F, N, Xs, A).
combination(F, N, Xs) -> combination(F, N, Xs, []).

> yaep:combination(3, [a, b, c, d, e]).
[[a,b,c],
 [a,b,d],
 [a,b,e],
 [a,c,d],
 [a,c,e],
 [a,d,e],
 [b,c,d],
 [b,c,e],
 [b,d,e],
 [c,d,e]]

% ========================================================================================================== 
repeat_comb(0, _) -> [[]];
repeat_comb(N, [X]) -> [make_list(X, N)];
repeat_comb(N, [Y | Ys]) ->
  lists:map(fun(X) -> [Y | X] end,
            repeat_comb(N - 1, [Y | Ys])) ++ repeat_comb(N, Ys).

% 高階関数版
repeat_comb(F, 0, _, A) -> F(lists:reverse(A));
repeat_comb(F, N, [X], A) -> F(lists:reverse(A) ++ make_list(X, N));
repeat_comb(F, N, [X | Xs], A) ->
  repeat_comb(F, N - 1, [X | Xs], [X | A]), repeat_comb(F, N, Xs, A).
repeat_comb(F, N, Xs) -> repeat_comb(F, N, Xs, []).

> yaep:repeat_comb(3, [a, b, c, d]).
[[a,a,a],
 [a,a,b],
 [a,a,c],
 [a,a,d],
 [a,b,b],
 [a,b,c],
 [a,b,d],
 [a,c,c],
 [a,c,d],
 [a,d,d],
 [b,b,b],
 [b,b,c],
 [b,b,d],
 [b,c,c],
 [b,c,d],
 [b,d,d],
 [c,c,c],
 [c,c,d],
 [c,d,d],
 [d,d,d]]
 
% ==========================================================================================================

split_nth(N, Xs) when N > 0 -> {take(N - 1, Xs), drop(N - 1, Xs)}.

> yaep:split_nth(3, [a, b, c, d, e, f]).
{[a,b],[c,d,e,f]}
> yaep:split_nth(4, [a, b, c, d, e, f]).
{[a,b,c],[d,e,f]}
> yaep:split_nth(1, [a, b, c, d, e, f]).
{[],[a,b,c,d,e,f]}
> yaep:split_nth(6, [a, b, c, d, e, f]).
{[a,b,c,d,e],[f]}
> yaep:split_nth(7, [a, b, c, d, e, f]).
{[a,b,c,d,e,f],[]}

% ==========================================================================================================
partition(_, []) -> {[], []};
partition(Pred, [X | Xs]) ->
  {A, B} = partition(Pred, Xs),
  case Pred(X) of
    true -> {[X | A], B};
    false -> {A, [X | B]}
  end.

% 別解
partition1(Pred, Xs) -> {[X || X <- Xs, Pred(X)], [X || X <- Xs, not Pred(X)]}.

partition2(Pred, Xs) ->
  Ys = [X || X <- Xs, Pred(X)], {Ys, difference(Xs, Ys)}.

> yaep:partition(fun(X) -> X rem 2 =:= 0 end, [1, 2, 3, 4, 5, 6, 7, 8]).
{[2,4,6,8],[1,3,5,7]}

% substitute second arg or function for first arg
% ==========================================================================================================
substitute(_, _, []) -> [];
substitute(X, Y, [Y | Ys]) -> [X | substitute(X, Y, Ys)];
substitute(X, Y, [Z | Zs]) -> [Z | substitute(X, Y, Zs)].

substitute_if(_, _, []) -> [];
substitute_if(X, Pred, [Y | Ys]) ->
  case Pred(Y) of
    true -> [X | substitute_if(X, Pred, Ys)];
    false -> [Y | substitute_if(X, Pred, Ys)]
  end.
  
> yaep:substitute(a, b, [a, b, c, a, b, c, a, b, c]).
[a,a,c,a,a,c,a,a,c]
> yaep:substitute_if(a, fun(X) -> X rem 2 =:= 0 end, [1, 2, 3, 4, 5, 6, 7, 8]).
[1,a,3,a,5,a,7,a]

% join the two lists as pairs
% ==========================================================================================================
zip([], _) -> [];
zip(_, []) -> [];
zip([X | Xs], [Y | Ys]) -> [{X, Y} | zip(Xs, Ys)].

> yaep:zip([a, b, c, d, e], [1, 2, 3, 4, 5]).
[{a,1},{b,2},{c,3},{d,4},{e,5}]

% separate the values into lists
% ==========================================================================================================
pack([], Ys, A) -> lists:reverse([Ys | A]);
pack([X | Xs], [X | Ys], A) -> pack(Xs, [X, X | Ys], A);
pack([X | Xs], Ys, A) -> pack(Xs, [X], [Ys | A]).

pack([X | Xs]) -> pack(Xs, [X], []).

 yaep:pack([a, a, a, b, b, c, d, d, d, d, e, e, e, e, e, e]).
[[a,a,a],[b,b],[c],[d,d,d,d],[e,e,e,e,e,e]]

% set of counts and the letter
% ==========================================================================================================
encode(Ys) -> [{X, length(Xs) + 1} || [X | Xs] <- pack(Ys)].

> yaep:encode([a, a, a, b, c, c, c, c, d, d, e, e, e]).
[{a,3},{b,1},{c,4},{d,2},{e,3}]

% set of counts and the letter
% ==========================================================================================================
decode([]) -> [];
decode([{X, L} | Xs]) -> make_list(X, L) ++ decode(Xs).

> yaep:decode([{a, 3}, {b, 1}, {c, 4}, {d, 2}, {e, 3}]).
[a,a,a,b,c,c,c,c,d,d,e,e,e]

% for eaxh list perform the functional code described
% ==========================================================================================================
for_each_list(_, _, Term, []) -> Term;
for_each_list(F, Comb, Term, [X | Xs]) ->
    Comb(F(X), for_each_list(F, Comb, Term, Xs)).
	
> yaep:for_each_list(fun(X) -> X end, fun(X,Y) -> X + Y end, 0, [1, 2, 3, 4, 5]).
15
> yaep:for_each_list(fun(X) -> X * X end, fun(X,Y) -> X + Y end, 0, [1, 2, 3, 4, 5]).
55
> yaep:for_each_list(fun(X) -> X end, fun lists:append/2, [], [[1, 2], [3], [4, 5]]).
[1,2,3,4,5]
> yaep:for_each_list(fun(X) -> X rem 2 =:= 0 end, 0, [1, 2, 3, 4, 5, 6, 7, 8]).
[2,4,6,8]
% example is x1 prio1 = 1 p2 = 2 p3 = 3 p4 =4 then x2 p1 = 5 etc... here we pull out all p3 elements
> yaep:for_each_list(fun(X) -> X rem 4 =:= 3 end, 0, [1, 2, 3, 4, 5, 6, 7, 8, 9. 10, 11, 12]).
[3,7,11]
% example is x1 prio1 = 1 p3 = 2 p3 = 3 p4 =4 then x2 p1 = 5 etc here we pull out all p4 elements and make a list of the x units
> yaep:for_each_list(fun(X) -> X rem 4 =:= 0 end, fun(X) -> X / 4 end, 0, [1, 2, 3, 4, 5, 6, 7, 8, 9. 10, 11, 12]).
[1,2,3]
> yaep:for_each_list(fun(X) -> X rem 4 =:= 0 end, fun(X) -> X / 4 end, 0, [1, 16, 4, 5, 6, 7, 12]).
[4,1,3]
% example is x1 prio1 = 1 p2 = 2 p3 = 3 p4 =4 then x2 p1 = 5 etc... here we pull out all p3 elements and normalising the x unit
> yaep:for_each_list(fun(X) -> X rem 4 =:= 3 end, fun(X) -> (X + 1) / 4 end, 0, [7, 3, 9, 1, 11]).
[2,1,3]
% example is x1 prio1 = 1 p2 = 2 p3 = 3 p4 =4 then x2 p1 = 5 etc... here we pull out all p2 elements and normalising the x unit
> yaep:for_each_list(fun(X) -> X rem 4 =:= 2 end, fun(X) -> (X + 2) / 4 end, 0, [7, 3, 9, 2, 11]).
[1]
% example is x1 prio1 = 1 p2 = 2 p3 = 3 p4 =4 then x2 p1 = 5 etc... here we pull out all p1 elements and normalising the x unit
> yaep:for_each_list(fun(X) -> X rem 4 =:= 1 end, fun(X) -> (X + 3) / 4 end, 0, [1, 3, 9, 2, 11, 13]).
[1,3,4]

% merge the numbers and sort them in the order specified by the function
% ==========================================================================================================
merge_sort(_, 0, _) -> [];
merge_sort(_, 1, [X | _]) -> [X];
merge_sort(P, 2, [X, Y | _]) ->
  case P(X, Y) of
    true -> [X, Y];
    false -> [Y, X]
  end;
merge_sort(P, N, Xs) ->
  M = N div 2,
  merge_list(P, merge_sort(P, M, Xs), merge_sort(P, N - M, drop(M, Xs))).
  
> yaep:merge_sort(fun(X,Y)-> X < Y end, 9, [5, 6, 4, 7, 3, 8, 2, 9, 1]).
[1,2,3,4,5,6,7,8,9]
> yaep:merge_sort(fun(X,Y)-> X < Y end, 9, [9, 8, 7, 6, 5, 4, 3, 2, 1]).
[1,2,3,4,5,6,7,8,9]
> yaep:merge_sort(fun(X,Y)-> X < Y end, 10, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]).
[1,2,3,4,5,6,7,8,9,10]

% merge lists in a unique union
% ==========================================================================================================
union([], Ys) -> Ys;
union([X | Xs], Ys) ->
  case lists:member(X, Ys) of
    true -> union(Xs, Ys);
    false -> [X | union(Xs, Ys)]
  end.
  
> yaep:union([a, b, c, d], [c, d, e, f]).
[a,b,c,d,e,f]
> yaep:union([a, b, c, d], [e, f, g, h]).
[a,b,c,d,e,f,g,h]
> yaep:union([a, b, c, d], [a, b, c, d]).
[a,b,c,d]

% Define a function last(Xs) to find the last element of the list, and a function to remove the last element to butlast(Xs).
% ==========================================================================================================
last([X]) -> X;
last([_ | Xs]) -> last(Xs).

> yaep:last([a, b, c]).
c

% Define a function take(N, Xs) that retrieves N elements from the beginning of list Xs.
% ==========================================================================================================
take(0, _) -> [];
take(_, []) -> [];
take(N, [X | Xs]) when N > 0 -> [X | take(N - 1, Xs)].

> yaep:take(3, [a, b, c, d, e]).
[a,b,c]
> yaep:take(0, [a, b, c, d, e]).
[]
> yaep:take(5, [a, b, c, d, e]).
[a,b,c,d,e]
> yaep:take(6, [a, b, c, d, e]).
[a,b,c,d,e]


% Drop the first N elemenets from the list
% ==========================================================================================================
drop(0, Xs) -> Xs;
drop(_, []) -> [];
drop(N, [_ | Xs]) when N > 0 -> drop(N - 1, Xs).

> yaep:drop(3, [a, b, c, d, e]).
[d,e]
> yaep:drop(0, [a, b, c, d, e]).
[a,b,c,d,e]
> yaep:drop(5, [a, b, c, d, e]).
[]
> yaep:drop(6, [a, b, c, d, e]).
[]


% create a subsequence 
% ========================================================================================================== 
subseq(N, M, Xs) when N > 0, M >= N -> take(M - N, drop(N - 1, Xs)).

> yaep:subseq(1, 3, [a, b, c, d, e]).
[a,b]
> yaep:subseq(1, 6, [a, b, c, d, e]).
[a,b,c,d,e]
> yaep:subseq(3, 6, [a, b, c, d, e]).
[c,d,e]
> yaep:subseq(3, 3, [a, b, c, d, e]).
[]

% find max or min in the list
% ==========================================================================================================
max_list([], Max) -> Max;
max_list([X | Xs], Max) when X > Max -> max_list(Xs, X);
max_list([_ | Xs], Max) -> max_list(Xs, Max).
max_list([X | Xs]) -> max_list(Xs, X).

min_list([], Min) -> Min;
min_list([X | Xs], Min) when X < Min -> min_list(Xs, X);
min_list([_ | Xs], Min) -> min_list(Xs, Min).
min_list([X | Xs]) -> min_list(Xs, X).

> yaep:max_list([5, 6, 4, 7, 3, 8, 2, 9, 1]).
9
> yaep:min_list([5, 6, 4, 7, 3, 8, 2, 9, 1]).
1

% intersection
% ==========================================================================================================
intersection([], _) -> [];
intersection([X | Xs], Ys) ->
  case lists:member(X, Ys) of
    true -> [X | intersection(Xs, Ys)];
    false -> intersection(Xs, Ys)
  end.
  
> yaep:intersection([a, b, c, d], [c, d, e, f]).
[c,d]
> yaep:intersection([a, b, c, d], [e, f, g, h]).
[]
> yaep:intersection([a, b, c, d], [a, b, c, d]).
[a,b,c,d]

% difference
% ==========================================================================================================
difference([], _) -> [];
difference([X | Xs], Ys) ->
  case lists:member(X, Ys) of
    true -> difference(Xs, Ys);
    false -> [X | difference(Xs, Ys)]
  end.
  
> yaep:difference([a, b, c, d], [c, d, e, f]).
[a,b]
> yaep:difference([a, b, c, d], [e, f, g, h]).
[a,b,c,d]
> yaep:difference([a, b, c, d], [a, b, c, d]).
[]

% merge 2 lists
% ==========================================================================================================
merge_list(_, [], Ys) -> Ys;
merge_list(_, Xs, []) -> Xs;
merge_list(P, [X | Xs], [Y | Ys]) ->
  case P(X, Y) of
    true -> [X | merge_list(P, Xs, [Y | Ys])];
    false -> [Y | merge_list(P, [X | Xs], Ys)]
  end.

> yaep:merge_list(fun(X,Y)-> X < Y end, [1, 3, 5, 8], [2, 4, 6, 7]).
[1,2,3,4,5,6,7,8]
> yaep:merge_list(fun(X,Y)-> X < Y end, [1, 3, 5, 7], [1, 3, 5, 7, 9]).
[1,1,3,3,5,5,7,7,9]
> yaep:merge_list(fun(X,Y)-> X < Y end, [2, 4, 6], [2, 4, 6, 8]).
[2,2,4,4,6,6,8]

% sum the list
% ==========================================================================================================
sum_list([], A) -> A;
sum_list([X | Xs], A) -> sum_list(Xs, A + X).
sum_list(Xs) -> sum_list(Xs, 0).

> yaep:sum_list([1, 2, 3, 4, 5, 6, 7, 8]).
36
> yaep:sum_list([1, -2, 3, -4, 5, -6, 7, -8]).
-4

% position of N in list
% ==========================================================================================================
position_if(_, _, []) -> false;
position_if(P, N, [X | Xs]) ->
  case P(X) of
    true -> N;
    false -> position_if(P, N + 1, Xs)
  end.
position_if(P, Xs) -> position_if(P, 1, Xs).

> yaep:position_if(fun(X) -> X =:= 2 end, [6, 5, 4, 3, 2, 1]).
5
> yaep:position_if(fun(X) -> X =:= 6 end, [6, 5, 4, 3, 2, 1]).
1
> yaep:position_if(fun(X) -> X =:= 1 end, [6, 5, 4, 3, 2, 1]).
6
> yaep:position_if(fun(X) -> X =:= 0 end, [6, 5, 4, 3, 2, 1]).
false

% count quantity if expression is true
% ==========================================================================================================
count_if(_, C, []) -> C;
count_if(P, C, [X | Xs]) ->
  case P(X) of
    true -> count_if(P, C + 1, Xs);
    false -> count_if(P, C, Xs)
  end.
count_if(P, Xs) -> count_if(P, 0, Xs).

> yaep:count_if(fun(X) -> X =:= 1 end, [1, 1, 2, 1, 2, 3, 1, 2, 3, 4]).
4
> yaep:count_if(fun(X) -> X =:= 2 end, [1, 1, 2, 1, 2, 3, 1, 2, 3, 4]).
3
> yaep:count_if(fun(X) -> X =:= 3 end, [1, 1, 2, 1, 2, 3, 1, 2, 3, 4]).
2
> yaep:count_if(fun(X) -> X =:= 4 end, [1, 1, 2, 1, 2, 3, 1, 2, 3, 4]).
1
> yaep:count_if(fun(X) -> X =:= 5 end, [1, 1, 2, 1, 2, 3, 1, 2, 3, 4]).
0

% X Y are adjacent in the list or not?
% ==========================================================================================================
adjacent(_, _, [_]) -> false;
adjacent(X, Y, [X, Y | _]) -> true;
adjacent(X, Y, [_ | Xs]) -> adjacent(X, Y, Xs).

> yaep:adjacent(a, b, [a, b, c, d, e]).
true
> yaep:adjacent(d, e, [a, b, c, d, e]).
true
> yaep:adjacent(a, c, [a, b, c, d, e]).
false
> yaep:adjacent(e, d, [a, b, c, d, e]).
false

% X Y are before in the list or not?
% ==========================================================================================================
before(_, _, []) -> false;
before(X, Y, [X | Xs]) -> lists:member(Y, Xs);
before(X, Y, [_ | Xs]) -> before(X, Y, Xs).

> yaep:before(a, b, [a, b, c, d, e]).
true
> yaep:before(a, e, [a, b, c, d, e]).
true
> yaep:before(c, b, [a, b, c, d, e]).
false
> yaep:before(e, a, [a, b, c, d, e]).
false

% sub X with Y
% ==========================================================================================================
subst(X, Y, X) -> Y;
subst(X, Y, [Z | Zs]) -> [subst(X, Y, Z) | subst(X, Y, Zs)];
subst(_, _, Z) -> Z.

subst_cps(X, Y, X, Cont) -> Cont(Y);
subst_cps(X, Y, [Z | Zs], Cont) ->
  subst_cps(X, Y, Z, fun(A) -> subst_cps(X, Y, Zs, fun(B) -> Cont([A | B]) end) end);
subst_cps(_, _, Z, Cont) -> Cont(Z).
subst_cps(X, Y, Xs) -> subst_cps(X, Y, Xs, fun(Z) -> Z end).

> yaep:subst(a, x, [a, [b, [c, [d | a], f], a], h]).
[x,[b,[c,[d|x],f],x],h]
> yaep:subst_cps(a, x, [a, [b, [c, [d | a], f], a], h], fun(X) -> X end).
[x,[b,[c,[d|x],f],x],h]

% quick sort the list
% ==========================================================================================================
quick_sort(_, []) -> [];
quick_sort(Pred, [X | Xs]) ->
  {A, B} = partition(fun(Y) -> Pred(Y, X) end, Xs),
  quick_sort(Pred, A) ++ [X] ++ quick_sort(Pred, B).
  
> yaep:quick_sort(fun(X, Y) -> X < Y end, [5, 6, 4, 7, 3, 8, 2, 9, 1, 0]).
[0,1,2,3,4,5,6,7,8,9]
> yaep:quick_sort(fun(X, Y) -> X > Y end, [5, 6, 4, 7, 3, 8, 2, 9, 1, 0]).
[9,8,7,6,5,4,3,2,1,0]

% In the path diagram below, create a program to find the route from the start (A) to the goal (G) using "depth-first search".
%  B D F
%A
%  C E G
% ==========================================================================================================
adjacent(a) -> [b, c];
adjacent(b) -> [a, c, d];
adjacent(c) -> [a, b, e];
adjacent(d) -> [b, e, f];
adjacent(e) -> [c, d, g];
adjacent(f) -> [d];
adjacent(g) -> [e].

% depth-first search
dfs(G, [G | Path]) ->
  io:write(lists:reverse([G | Path])), io:nl();
dfs(G, [X | Path]) ->
  lists:foreach(
    fun (Y) ->
      case lists:member(Y, Path) of
        true -> false;
        false -> dfs(G, [Y, X | Path])
      end
    end,
    adjacent(X)).
  
> yaep:dfs(g, [a]).
[a,b,c,e,g]
[a,b,d,e,g]
[a,c,b,d,e,g]
[a,c,e,g]
ok

> yaep:dfs(d, [a]).
[a,b,d]
[a,c,b,d]
[a,c,e,d]
ok

% In the path diagram below, create a program to find the route from the start (A) to the goal (G) using "bredth-first search".
%  B D F
%A
%  C E G
% ==========================================================================================================
adjacent(a) -> [b, c];
adjacent(b) -> [a, c, d];
adjacent(c) -> [a, b, e];
adjacent(d) -> [b, e, f];
adjacent(e) -> [c, d, g];
adjacent(f) -> [d];
adjacent(g) -> [e].

bfs(_, []) -> ok;
bfs(Goal, [[Goal | Xs] | Ys]) ->
  io:write(lists:reverse([Goal | Xs])), io:nl(), bfs(Goal, Ys);
bfs(Goal, [[X | Xs] | Ys]) ->
  bfs(Goal, Ys ++ lists:foldl(fun(Y, A) -> 
                                case lists:member(Y, Xs) of
                                  true -> A;
                                  false -> [[Y, X | Xs] | A]
                                end
                              end,
                              [],
                              adjacent(X))).
							  
> yaep:bfs(g, [[a]]).
[a,c,e,g]
[a,b,d,e,g]
[a,b,c,e,g]
[a,c,b,d,e,g]
ok

% In the path diagram below, create a program to find the route from the start (A) to the goal (G) using "iterative deepening".
%  B D F
%A
%  C E G
% ==========================================================================================================
iota(N, M) when N > M -> [];
iota(N, M) -> [N | iota(N + 1, M)].

iota(N, N, A) -> [N | A];
iota(N, M, A) -> iota(N, M - 1, [M | A]).
iota(N, M) -> iota(N, M, []).

> yaep:iota(1, 8).
[1,2,3,4,5,6,7,8]
> yaep:iota(1, 1).
[1]
> yaep:iota(1, 0).
[]

adjacent(a) -> [b, c];
adjacent(b) -> [a, c, d];
adjacent(c) -> [a, b, e];
adjacent(d) -> [b, e, f];
adjacent(e) -> [c, d, g];
adjacent(f) -> [d];
adjacent(g) -> [e].

ids(Limit, Goal, Path) when length(Path) =:= Limit ->
  if
    hd(Path) =:= Goal -> io:write(lists:reverse(Path)), io:nl();
    true -> false
  end;
ids(Limit, Goal, Path) ->
  lists:foreach(fun(N) ->
                  case lists:member(N, Path) of
                    true -> false;
                    false -> ids(Limit, Goal, [N | Path])
                  end
                end,
                adjacent(hd(Path))).

ids(Start, Goal) ->
  lists:foreach(fun(Limit) -> ids(Limit, Goal, [Start]) end, iota(1, 7)).
  
> yaep:ids(a, g).
[a,c,e,g]
[a,b,c,e,g]
[a,b,d,e,g]
[a,c,b,d,e,g]
ok