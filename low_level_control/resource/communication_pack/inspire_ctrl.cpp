#include "inspire.h"
#include "param.h"

#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>
#include <queue>
#include <chrono>

class InspireRunner
{
private:
    struct PositionHistory {
        Eigen::Matrix<double, 6, 1> q;
        std::chrono::time_point<std::chrono::high_resolution_clock> time;
    };

    std::queue<PositionHistory> right_hand_history;
    std::queue<PositionHistory> left_hand_history;
    const size_t HISTORY_SIZE = 5;

    void calculateVelocityAndAcceleration(
        const std::queue<PositionHistory>& history,
        Eigen::Matrix<double, 6, 1>& velocity,
        Eigen::Matrix<double, 6, 1>& acceleration) 
    {
        if (history.size() < 2) return;

        std::vector<PositionHistory> hist_vec;
        auto temp_history = history;
        while (!temp_history.empty()) {
            hist_vec.push_back(temp_history.front());
            temp_history.pop();
        }

        double dt_total = std::chrono::duration<double>(
            hist_vec.back().time - hist_vec.front().time).count();

        if (dt_total <= 0) return;

        for (int i = 0; i < 6; ++i) {
            velocity(i) = (hist_vec.back().q(i) - hist_vec.front().q(i)) / dt_total;

            if (hist_vec.size() >= 3) {
                double dt1 = std::chrono::duration<double>(
                    hist_vec[1].time - hist_vec[0].time).count();
                double dt2 = std::chrono::duration<double>(
                    hist_vec[2].time - hist_vec[1].time).count();

                double v_prev = (hist_vec[1].q(i) - hist_vec[0].q(i)) / dt1;
                double v_next = (hist_vec[2].q(i) - hist_vec[1].q(i)) / dt2;
                acceleration(i) = (v_next - v_prev) / (0.5*(dt1 + dt2));
            }
        }
    }

public:
    InspireRunner()
    {
        serial = std::make_shared<SerialPort>(param::serial_port, B115200);

        righthand = std::make_shared<inspire::InspireHand>(serial, 1);
        lefthand = std::make_shared<inspire::InspireHand>(serial, 2);

        const int16_t max_speed = 1000;
        righthand->SetVelocity(max_speed, max_speed, max_speed, max_speed, max_speed, max_speed);
        lefthand->SetVelocity(max_speed, max_speed, max_speed, max_speed, max_speed, max_speed);

        Eigen::Matrix<double, 6, 1> open_hand;
        open_hand << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        righthand->SetPosition(open_hand);
        lefthand->SetPosition(open_hand);

        handcmd = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>(
            "rt/" + param::ns + "/cmd");
        handcmd->msg_.cmds().resize(12);
        
        handstate = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>(
            "rt/" + param::ns + "/state");
        handstate->msg_.states().resize(12);

        thread = std::make_shared<unitree::common::RecurrentThread>(
            10000, std::bind(&InspireRunner::run, this)
        );
    }

    void run()
    {
        auto now = std::chrono::high_resolution_clock::now();

        // Установка команд
        for(int i = 0; i < 12; i++) {
            qcmd(i) = handcmd->msg_.cmds()[i].q();
        }
        righthand->SetPosition(qcmd.block<6, 1>(0, 0));
        lefthand->SetPosition(qcmd.block<6, 1>(6, 0));

        // Получение состояния и расчёт динамики
        Eigen::Matrix<double, 6, 1> qtemp, vtemp, atemp, ftemp;
        
        // Правая кисть: положение + сила
        if(righthand->GetPosition(qtemp) == 0) {
            right_hand_history.push({qtemp, now});
            if (right_hand_history.size() > HISTORY_SIZE) {
                right_hand_history.pop();
            }

            if (right_hand_history.size() >= 2) {
                calculateVelocityAndAcceleration(right_hand_history, vtemp, atemp);
                qstate.block<6, 1>(0, 0) = qtemp;
                vstate.block<6, 1>(0, 0) = vtemp;
                astate.block<6, 1>(0, 0) = atemp;
            }
        } else {
            for(int i = 0; i < 6; i++) {
                handstate->msg_.states()[i].lost()++;
            }
        }

        // Левая кисть: положение + сила
        if(lefthand->GetPosition(qtemp) == 0) {
            left_hand_history.push({qtemp, now});
            if (left_hand_history.size() > HISTORY_SIZE) {
                left_hand_history.pop();
            }

            if (left_hand_history.size() >= 2) {
                calculateVelocityAndAcceleration(left_hand_history, vtemp, atemp);
                qstate.block<6, 1>(6, 0) = qtemp;
                vstate.block<6, 1>(6, 0) = vtemp;
                astate.block<6, 1>(6, 0) = atemp;
            }
        } else {
            for(int i = 0; i < 6; i++) {
                handstate->msg_.states()[i+6].lost()++;
            }
        }

        // Получение сил для обеих кистей
        if(righthand->GetForce(ftemp) == 0) {
            fstate.block<6, 1>(0, 0) = ftemp;
        }
        if(lefthand->GetForce(ftemp) == 0) {
            fstate.block<6, 1>(6, 0) = ftemp;
        }

        // Публикация состояния
        if(handstate->trylock()) {
            for(int i = 0; i < 12; i++) {
                handstate->msg_.states()[i].q() = qstate(i);
                handstate->msg_.states()[i].dq() = vstate(i);
                handstate->msg_.states()[i].ddq() = astate(i);
                handstate->msg_.states()[i].tau_est() = fstate(i); // Сила (N)
            }
            handstate->unlockAndPublish();
        }
    }

    unitree::common::ThreadPtr thread;
    SerialPort::SharedPtr serial;
    std::shared_ptr<inspire::InspireHand> lefthand, righthand;
    
    Eigen::Matrix<double, 12, 1> qcmd;   // Команды
    Eigen::Matrix<double, 12, 1> qstate; // Положения
    Eigen::Matrix<double, 12, 1> vstate; // Скорости
    Eigen::Matrix<double, 12, 1> astate; // Ускорения
    Eigen::Matrix<double, 12, 1> fstate; // Силы (N)

    std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>> handstate;
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>> handcmd;
};

int main(int argc, char **argv)
{
    auto vm = param::helper(argc, argv);
    unitree::robot::ChannelFactory::Instance()->Init(0, param::network);

    std::cout << " --- Unitree Robotics --- " << std::endl;
    std::cout << "  Inspire Hand Controller  " << std::endl;

    InspireRunner runner;

    while (true) {
        sleep(1);
    }
    return 0;
}