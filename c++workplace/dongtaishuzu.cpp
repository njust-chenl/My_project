#include <iostream>

int main(){
    int arr[]={-7, -3 , 2 ,-1 , 1,-7,7,7 };
    int target[]={0};
    target[0]=arr[0];
    for(int i=1;i<8;i++){
        int a=target[i-1]+arr[i];
        int b=arr[i];
        target[i]=std::max(a,b);
    }

    
}