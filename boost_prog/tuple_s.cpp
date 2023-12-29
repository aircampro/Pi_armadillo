#include <cstdlib>
#include <string>
#include <iostream>
#include <tuple>
#include <climits>
#pragma GCC push_options
#pragma GCC optimize ("Ofast")
template <size_t, bool>
struct Castable
{
    template <class Type>
    operator Type& () const&& noexcept;
};
template <size_t S>
struct Castable<S, false>
{
    template <class Type>
    operator Type&& () const&& noexcept;
};

template <class Type, size_t ...Indices>
constexpr auto IsAggregateInitializable(std::index_sequence<Indices...>)
    -> decltype(Type{ Castable<Indices, std::is_copy_constructible<Type>::value>()... }, std::true_type())
{
    return std::true_type();
}
template <class Type>
constexpr std::false_type IsAggregateInitializable(...) { return std::false_type(); }

template <class Type, size_t N>
constexpr std::enable_if_t<IsAggregateInitializable<Type>(std::make_index_sequence<N>()), size_t>
GetNumOfMemberVariables_rec(int)
{
    return N;
}
template <class Type, size_t N>
constexpr auto GetNumOfMemberVariables_rec(long)
{
    return GetNumOfMemberVariables_rec<Type, N - 1>(1);
}
template <class Type>
constexpr size_t GetNumOfMemberVariables()
{
    return GetNumOfMemberVariables_rec<Type, sizeof(Type) * CHAR_BIT>(1);
}

template <class Type>
auto BindToTuple(Type& t, std::integral_constant<size_t, 1>)
{
    auto& [a] = t;
    return std::tie(a);
}
template <class Type>
auto BindToTuple(Type& t, std::integral_constant<size_t, 2>)
{
    auto& [a, b] = t;
    return std::tie(a, b);
}
template <class Type>
auto BindToTuple(Type& t, std::integral_constant<size_t, 3>)
{
    auto& [a, b, c] = t;
    return std::tie(a, b, c);
}
template <class Type>
auto BindToTuple(Type& t, std::integral_constant<size_t, 4>)
{
    auto& [a, b, c, d] = t;
    return std::tie(a, b, c, d);
}

template <class Type>
auto BindToTuple(Type& t, std::integral_constant<size_t, 5>)
{
    auto& [a, b, c, d, e] = t;
    return std::tie(a, b, c, d, e);
}
template <class Type>
auto BindToTuple(Type& t, std::integral_constant<size_t, 6>)
{
    auto& [a, b, c, d, e, f] = t;
    return std::tie(a, b, c, d, e, f);
}

template <size_t N, class Type>
decltype(auto) Get(Type& t)
{
   return std::get<N>(BindToTuple(t, std::integral_constant<size_t, GetNumOfMemberVariables<Type>()>()));
}

struct Test
{
    int x;
    std::string y;
    float f;
    std::string ss;
    double ff;
};
int main()
{
    Test test{ 1, "2.34", 2.00034, "99.5", -0.000032445 };
    constexpr size_t Size = GetNumOfMemberVariables<Test>();
    std::cout << "The number of Member variables is " << Size << std::endl;//2
    std::cout << Get<0>(test) << std::endl;//2.34
    std::cout << Get<1>(test) << std::endl;//2.34
    std::cout << Get<2>(test) << std::endl;//2.34
    std::cout << Get<3>(test) << std::endl;//2.34
    std::cout << Get<4>(test) << std::endl;//2.34
    return 0;
}

#pragma GCC pop_options
