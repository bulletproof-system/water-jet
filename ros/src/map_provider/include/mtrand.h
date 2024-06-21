// mtrand.h
// C++ include file for MT19937, with initialization improved 2002/1/26.
// Coded by Takuji Nishimura and Makoto Matsumoto.
// Ported to C++ by Jasper Bedaux 2003/1/1 (see http://www.bedaux.net/mtrand/).
// The generators returning floating point numbers are based on
// a version by Isaku Wada, 2002/01/09
//
// Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// 3. The names of its contributors may not be used to endorse or promote
//    products derived from this software without specific prior written
//    permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Any feedback is very welcome.
// http://www.math.keio.ac.jp/matumoto/emt.html
// email: matumoto@math.keio.ac.jp
//
// Feedback about the C++ port should be sent to Jasper Bedaux,
// see http://www.bedaux.net/mtrand/ for e-mail address and info.

#ifndef MTRAND_H
#define MTRAND_H

class MTRand_int32 { // 梅森旋转算法随机数生成器
public:
    // 默认构造函数：仅在这是第一个实例时使用默认种子
    MTRand_int32() {
        if (!init) seed(5489UL);
        init = true;
    }

    // 使用32位整数作为种子的构造函数
    MTRand_int32(unsigned long s) {
        seed(s);
        init = true;
    }

    // 使用32位整数数组作为种子的构造函数
    MTRand_int32(const unsigned long* array, int size) {
        seed(array, size);
        init = true;
    }

    // 两个种子函数
    void seed(unsigned long); // 使用32位整数作为种子
    void seed(const unsigned long*, int size); // 使用数组作为种子

    // 重载operator()以使其成为生成器（仿函数）
    unsigned long operator()() {
        return rand_int32();
    }

    // 将析构函数设为虚函数
    virtual ~MTRand_int32() {} // 析构函数

protected: // 供派生类使用，否则不可访问；使用()-操作符
    unsigned long rand_int32(); // 生成32位随机整数

private:
    static const int n = 624, m = 397; // 编译时常量
    // 以下变量是静态的（不能有重复）
    static unsigned long state[n]; // 状态向量数组
    static int p; // 状态数组中的位置
    static bool init; // 如果调用了init函数则为true

    // 用于生成伪随机数的私有函数
    unsigned long twiddle(unsigned long, unsigned long); // 被gen_state()使用
    void gen_state(); // 生成新状态

    // 使复制构造函数和赋值操作符不可用，它们没有意义
    MTRand_int32(const MTRand_int32&); // 复制构造函数未定义
    void operator=(const MTRand_int32&); // 赋值操作符未定义
};

// 为了速度，内联函数必须在头文件中
inline unsigned long MTRand_int32::twiddle(unsigned long u, unsigned long v) {
    return (((u & 0x80000000UL) | (v & 0x7FFFFFFFUL)) >> 1)
        ^ ((v & 1UL) * 0x9908B0DFUL);
    // 根据 http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/Ierymenko.html 进行性能优化
}

inline unsigned long MTRand_int32::rand_int32() { // 生成32位随机整数
    if (p == n) gen_state(); // 需要新的状态向量
    // gen_state()被分离出来为非内联，因为它每624次调用中只调用一次
    // 否则rand_int32()会变得太大而无法内联
    unsigned long x = state[p++];
    x ^= (x >> 11);
    x ^= (x << 7) & 0x9D2C5680UL;
    x ^= (x << 15) & 0xEFC60000UL;
    return x ^ (x >> 18);
}

// 生成半开区间[0, 1)内的双精度浮点数
class MTRand : public MTRand_int32 {
public:
    MTRand() : MTRand_int32() {}
    MTRand(unsigned long seed) : MTRand_int32(seed) {}
    MTRand(const unsigned long* seed, int size) : MTRand_int32(seed, size) {}
    ~MTRand() {}

    double operator()() {
        return static_cast<double>(rand_int32()) * (1. / 4294967296.); // 除以2^32
    }

private:
    MTRand(const MTRand&); // 复制构造函数未定义
    void operator=(const MTRand&); // 赋值操作符未定义
};

// 生成闭区间[0, 1]内的双精度浮点数
class MTRand_closed : public MTRand_int32 {
public:
    MTRand_closed() : MTRand_int32() {}
    MTRand_closed(unsigned long seed) : MTRand_int32(seed) {}
    MTRand_closed(const unsigned long* seed, int size) : MTRand_int32(seed, size) {}
    ~MTRand_closed() {}

    double operator()() {
        return static_cast<double>(rand_int32()) * (1. / 4294967295.); // 除以2^32 - 1
    }

private:
    MTRand_closed(const MTRand_closed&); // 复制构造函数未定义
    void operator=(const MTRand_closed&); // 赋值操作符未定义
};

// 生成开区间(0, 1)内的双精度浮点数
class MTRand_open : public MTRand_int32 {
public:
    MTRand_open() : MTRand_int32() {}
    MTRand_open(unsigned long seed) : MTRand_int32(seed) {}
    MTRand_open(const unsigned long* seed, int size) : MTRand_int32(seed, size) {}
    ~MTRand_open() {}

    double operator()() {
        return (static_cast<double>(rand_int32()) + .5) * (1. / 4294967296.); // 除以2^32
    }

private:
    MTRand_open(const MTRand_open&); // 复制构造函数未定义
    void operator=(const MTRand_open&); // 赋值操作符未定义
};

// 生成半开区间[0, 1)内的53位精度双精度浮点数
class MTRand53 : public MTRand_int32 {
public:
    MTRand53() : MTRand_int32() {}
    MTRand53(unsigned long seed) : MTRand_int32(seed) {}
    MTRand53(const unsigned long* seed, int size) : MTRand_int32(seed, size) {}
    ~MTRand53() {}

    double operator()() {
        return (static_cast<double>(rand_int32() >> 5) * 67108864. +
            static_cast<double>(rand_int32() >> 6)) * (1. / 9007199254740992.);
    }

private:
    MTRand53(const MTRand53&); // 复制构造函数未定义
    void operator=(const MTRand53&); // 赋值操作符未定义
};

#endif // MTRAND_H
