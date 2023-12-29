#include <iostream>
#include <algorithm>
#include <vector>
#include <numeric>
#include <map>
#include <random>
int main()
{
  using namespace std;
  string a = "wolf run";
  string b = "the wolfs";
  //string b = "paragraph";
  string tmp;
  vector<string> x;
  vector<string> y;
  int count_ = 0;
  for (int i = 0; i < a.size() - 1; i++)
  {
    tmp = a[i];
    tmp += a[i + 1];
    auto itr = find(x.begin(), x.end(), tmp); //There are duplicates in x, so this is excluded.
    if (itr == x.end())
      x.push_back(tmp);
  }
  for (int i = 0; i < b.size() - 1; i++)
  {
    tmp = b[i];
    tmp += b[i + 1];
    auto itr = find(y.begin(), y.end(), tmp); // as above
    if (itr == y.end())
      y.push_back(tmp);
  }

  //sum
  vector<string> wa;
  wa = x;
  for (int i = 0; i < y.size(); i++)
  {
    auto itr = find(x.begin(), x.end(), y[i]);
    if (itr == x.end())
    {
      wa.push_back(y[i]);
    }
  }
  cout << "***sum***" << endl;
  for (int i = 0; i < wa.size(); i++)
  {
    cout << wa[i] << endl;
  }
  //product
  vector<string> seki;
  for (int i = 0; i < y.size(); i++)
  {
    auto itr = find(x.begin(), x.end(), y[i]);
    if (itr != x.end())
    {
      seki.push_back(y[i]);
    }
  }
  cout << "***product***" << endl;
  for (int i = 0; i < seki.size(); i++)
  {
    cout << seki[i] << endl;
  }
  //difference
  vector<string> sa;
  for (int i = 0; i < x.size(); i++)
  {
    auto itr = find(y.begin(), y.end(), x[i]);
    if (itr == y.end())
    {
      sa.push_back(x[i]);
    }
  }
  cout << "***difference***" << endl;
  for (int i = 0; i < sa.size(); i++)
  {
    cout << sa[i] << endl;
  }

  if (find(x.begin(), x.end(), "wo") != x.end() && find(y.begin(), y.end(), "wo") != y.end())
    cout << "True" << endl;
  else
    cout << "False" << endl;
}
