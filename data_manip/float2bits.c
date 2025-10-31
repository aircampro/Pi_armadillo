// shows how to write a float as an int without union and write it to bits
#include <iostream>

int main() {
    // float to int using typecast pointer
    float f = -12.5f;
    // to convert
    int i = *( ( int* )&f );
    printf( "%f ( %08X )\n", f, i );
    for( int j = 31; j >= 0; j-- ){
        printf( "%d", ( i >> j ) & 1 );
    }
    printf( "\n" );
    // and back
    float z = *( ( float* )&i );
    printf( "%f\n", z );
    
    // float to int using union
    union { float f; int i; } a;
    a.f = -2.5f;
    printf( "%f ( %08X )\n", a.f, a.i );
    return 0;    
 
}
