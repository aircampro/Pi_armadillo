#include <iostream>
#include <string>
#include <algorithm>
using namespace std;
int LP[1005][1005]={};

int levenstein( std::string x, std::string y ) {
  int j,k;
  std::transform(x.begin(), x.end(),x.begin(), ::toupper);
  std::transform(y.begin(), y.end(),y.begin(), ::toupper);

  for(j=1;j<=x.size();j++) LP[j][0] = j;
  for(k=1;k<=y.size();k++) LP[0][k] = k;

  //I want a closer to b!
  for(j=1;j<=x.size();j++) {
    for(k=1;k<=y.size();k++) {
      //Remove a[j] or insert the same character as b[k] in a[j+1]
      //Adopt the minimum number of acts of the above 2
      int m = min(LP[j-1][k]+1, LP[j][k-1]+1);
      if(x[j-1] == y[k-1]) {
        // Since the last character is the same, the editing distance is 
        m = min(m,LP[j-1][k-1]);
        LP[j][k] = m;
      }else {
        // Replace last character
        m = min(m,LP[j-1][k-1]+1);
        LP[j][k] = m;
      }
    }
  }
  cout << LP[x.size()][y.size()] << endl;
  return LP[x.size()][y.size()];
}

int main(void) {

  int l_value = levenstein( "aperture", "aperture__1" );
  std::cout << "leven " << l_value << std::endl;

  l_value = levenstein( "aperture", "APERTUR" );
  std::cout << "leven " << l_value << std::endl;
  l_value = levenstein( "aperture", "NOTIT999" );
  std::cout << "leven " << l_value << std::endl;
}
