// mtrand.cpp, 详细信息请参见头文件 mtrand.h

#include "mtrand.h"

// 非内联函数定义和静态成员定义不能位于头文件中，以避免多重声明的风险

// 初始化静态私有成员
unsigned long MTRand_int32::state[n] = {0x0UL};
int MTRand_int32::p = 0;
bool MTRand_int32::init = false;

// 生成新的状态向量
void MTRand_int32::gen_state() {
    for (int i = 0; i < (n - m); ++i) {
        state[i] = state[i + m] ^ twiddle(state[i], state[i + 1]);
    }
    for (int i = n - m; i < (n - 1); ++i) {
        state[i] = state[i + m - n] ^ twiddle(state[i], state[i + 1]);
    }
    state[n - 1] = state[m - 1] ^ twiddle(state[n - 1], state[0]);
    p = 0; // 重置位置
}

// 通过32位种子初始化
void MTRand_int32::seed(unsigned long s) {
    state[0] = s & 0xFFFFFFFFUL; // 适用于大于32位的机器
    for (int i = 1; i < n; ++i) {
        state[i] = 1812433253UL * (state[i - 1] ^ (state[i - 1] >> 30)) + i;
        // 参见Knuth TAOCP Vol2. 3rd Ed. P.106中的乘数
        // 在之前的版本中，种子的MSB（最高有效位）只影响数组状态的MSB
        state[i] &= 0xFFFFFFFFUL; // 适用于大于32位的机器
    }
    p = n; // 强制在下一个随机数生成时调用gen_state()
}

// 通过数组初始化
void MTRand_int32::seed(const unsigned long* array, int size) {
    seed(19650218UL);
    int i = 1, j = 0;
    for (int k = ((n > size) ? n : size); k; --k) {
        state[i] = (state[i] ^ ((state[i - 1] ^ (state[i - 1] >> 30)) * 1664525UL))
                 + array[j] + j; // 非线性
        state[i] &= 0xFFFFFFFFUL; // 适用于大于32位的机器
        ++j; 
        j %= size;
        if ((++i) == n) { 
            state[0] = state[n - 1]; 
            i = 1; 
        }
    }
    for (int k = n - 1; k; --k) {
        state[i] = (state[i] ^ ((state[i - 1] ^ (state[i - 1] >> 30)) * 1566083941UL)) - i;
        state[i] &= 0xFFFFFFFFUL; // 适用于大于32位的机器
        if ((++i) == n) { 
            state[0] = state[n - 1]; 
            i = 1; 
        }
    }
    state[0] = 0x80000000UL; // MSB是1，确保初始数组非零
    p = n; // 强制在下一个随机数生成时调用gen_state()
}
