#include <stdio.h>

int swap_input(int input); 

int main(void) {

    int input = getchar();
    int i = 0;
    
    while (input != -1 && i < 1000) {
        int swapping_input = swap_input(input);
        putchar(swapping_input);
        input = getchar();
        
        i++;
        
    } 
    
    return 0;
    
}

int swap_input (int input) {
    
    // Capital Letters
    if (input >= 'A' && input <= 'Z') {
        // Lower Case
        input += 32;
    } else if (input >= 'a' && input <= 'z') {
        input -= 32;
    }
    
    return input;
}