#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>                // 用于设置终端属性
#include <stdio.h>

#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_SPACE 0x20          // 空格键

// 初始化终端属性
struct termios cooked, raw;

void quit(int sig) {
    // 恢复终端属性并退出
    tcsetattr(0, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

// 获取键盘输入
int kfd = 0;
void keyLoop(ros::Publisher& vel_pub) {
    char c;
    bool dirty = false;

    // 获取当前终端属性
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    // 设置终端为非规范模式，输入不会被缓冲
    raw.c_lflag &= ~(ICANON | ECHO);
    // 最小读取字符数为1
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    // 应用终端属性
    tcsetattr(kfd, TCSANOW, &raw);

    puts("使用方法：");
    puts("W - 向前加速");
    puts("S - 向后加速");
    puts("A - 向左加速");
    puts("D - 向右加速");
    puts("Q - 左旋加速");
    puts("E - 右旋加速");
    puts("Space - 停止所有运动");
    puts("---------------------------");
    puts("正在监听键盘控制……");

    for(;;) {
        // 非阻塞读取
        if(read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }

        geometry_msgs::Twist vel_cmd;
        switch(c) {
            case KEYCODE_W:
                vel_cmd.linear.x = 2.0;
                dirty = true;
                // puts("W - 向前加速");
                break;
            case KEYCODE_S:
                vel_cmd.linear.x = -2.0;
                dirty = true;
                // puts("S - 向后加速");
                break;
            case KEYCODE_A:
                vel_cmd.linear.y = 2.0;
                dirty = true;
                // puts("A - 向左加速");
                break;
            case KEYCODE_D:
                vel_cmd.linear.y = -2.0;
                dirty = true;
                // puts("D - 向右加速");
                break;
            case KEYCODE_Q:
                vel_cmd.angular.z = 2.0;
                dirty = true;
                // puts("Q - 左旋加速");
                break;
            case KEYCODE_E:
                vel_cmd.angular.z = -2.0;
                dirty = true;
                // puts("E - 右旋加速");
                break;
            case KEYCODE_SPACE:
                vel_cmd.linear.x = 0.0;
                vel_cmd.linear.y = 0.0;
                vel_cmd.angular.z = 0.0;
                dirty = true;
                // puts("Space - 停止所有运动");
                break;
            default:
                // 其他键被按下时不发送速度
                break;
        }

        if(dirty == true) {
            // 发布速度指令
            vel_pub.publish(vel_cmd);
            dirty = false;
        }
    }
    return;
}

int main(int argc, char** argv) {
    // 初始化节点
    ros::init(argc, argv, "vel_ctrl", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    signal(SIGINT, quit);

    keyLoop(vel_pub);

    return 0;
}
