/**
 * @file    Move3D.hpp
 * @author  Akano(Xiao Zhihao) (zxiaoav@connect.ust.hk)
 * @brief   用于3D运动建模的库
 * @version 1.1.1
 * 核心功能
 * 1.Movement
 * 统一 predict(double dt) 的接口，描述一个物体的运动，本身是一个黑箱函数
 * 提供了一些自带的运动模型，也可以手动继承构造Movement的子类描述运动。
 * 2.BallisticCalculator
 * 传入一个目标的运动Movement，解出该以何种姿态发出子弹才能击中目标。
 * 目前统一使用纯抛物线模型
 */
#include <iostream>
#include <cmath>
#include <utility>
#include <functional>

namespace Move3D{
    constexpr double EPS = 1e-10;
    constexpr double G_VALUE = 9.81;

    struct Vector{
        double x,y,z;
        Vector() = default;
        Vector(double _x,double _y,double _z){
            x = _x,y = _y,z = _z;
        }
        Vector operator*(double ratio) const {
            return Vector(x * ratio,y * ratio,z * ratio);
        }
        double GetLen() const {
            return sqrt(x * x + y * y + z * z);
        }
    };
    struct Point{
        double x,y,z;
        Point() = default;
        Point(double _x,double _y,double _z){
            x = _x,y = _y,z = _z;
        }
        Point operator+(Vector delta) const {
            return Point(this->x + delta.x,this->y + delta.y,this->z + delta.z);
        }
        Vector ToVector() const {
            return Vector(x,y,z);
        }
    };
    struct RPY{
        double roll,pitch,yaw;
        RPY() = default;
        RPY(double _roll,double _pitch,double _yaw){
            roll = _roll,pitch = _pitch,yaw = _yaw;
        }
    };

    inline Vector operator-(const Point& p1,const Point& p2){
        return Vector(p1.x - p2.x,p1.y - p2.y,p1.z - p2.z);
    }

    inline double GetDistance(const Point& p1,const Point& p2){
        return sqrt(
            (p1.x - p2.x) * (p1.x - p2.x) + 
            (p1.y - p2.y) * (p1.y - p2.y) + 
            (p1.z - p2.z) * (p1.z - p2.z)
        );
    }

    //WARNING:AI-GEN
    inline Vector RotateRPY(const Vector& v,const RPY& rpy){
        double cr = std::cos(rpy.roll),  sr = std::sin(rpy.roll);
        double cp = std::cos(rpy.pitch), sp = std::sin(rpy.pitch);
        double cy = std::cos(rpy.yaw),   sy = std::sin(rpy.yaw);

        // Rx(roll)
        double x1 = v.x;
        double y1 = v.y * cr - v.z * sr;
        double z1 = v.y * sr + v.z * cr;

        // Ry(pitch)
        double x2 = x1 * cp + z1 * sp;
        double y2 = y1;
        double z2 = -x1 * sp + z1 * cp;

        // Rz(yaw)
        double x3 = x2 * cy - y2 * sy;
        double y3 = x2 * sy + y2 * cy;
        double z3 = z2;

        return Vector(x3,y3,z3);
    }

    inline RPY VectorToRPY(const Vector& v){
        double len = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
        if(len <= EPS) return RPY(0,0,0);

        double vx = v.x / len,vy = v.y / len,vz = v.z / len;
        double yaw = atan2(vy,vx);
        double pitch = atan2(-vz,sqrt(vx*vx + vy*vy));

        return RPY(0,pitch,yaw);
    }

    class Movement{
    protected:
        Point nowPosition;
    public:
        virtual ~Movement() = default;
        virtual Point Predict(double dt) const = 0;
        Point GetNowPosition() const {
            return nowPosition;
        }
    };

    class UniformLinearMotion : public Movement{
    private:
        Vector velocity;
    public:
        UniformLinearMotion(Point _nowPosition,Vector _velocity){
            nowPosition = _nowPosition;
            velocity = _velocity;
        }
        Point Predict(double dt) const override {
            return nowPosition + velocity * dt;
        }
    };

    class UniformAcceleratedMotion : public Movement{
    private:
        Vector velocity,acceleration;
    public:
        UniformAcceleratedMotion(Point _nowPosition,Vector _velocity,Vector _acceleration){
            nowPosition = _nowPosition;
            velocity = _velocity;
            acceleration = _acceleration;
        }
        Point Predict(double dt) const override {
            return nowPosition + velocity * dt + acceleration * 0.5 * dt * dt;
        }
    };

    class BallisticCalculator{
    public:
        using ErrorFunc = std::function<Vector(double,double,double,double)>;
        struct AimResult{
            double pitch,yaw,t;
            Vector error;
            enum class ErrorCode : unsigned int{
                OK = 0,
                TIME_ERROR = 1,
                MODEL_ERROR = 2147483647
            }errorCode;
        };
        enum class BulletModel : unsigned int{
            light = 0,
            parabola = 1
        };
    private:
        constexpr static double minTime = 1e-4;
        int iterationTime;
        int tryTime;
        double lambdaDecay;
        double dp,dy,dt;
        AimResult GetPitchYawBasic(const Movement& mov,double speed,const ErrorFunc& evalError){
            auto Solve3x3 = [](double l[3][4]){
                for(int i = 0;i < 3;i++){
                    int swaptail = i;
                    for(int j = i+1;j < 3;j++){
                        if(fabsl(l[j][i]) > fabsl(l[swaptail][i])){
                            swaptail = j;
                        }
                    }
                    if(swaptail != i){
                        for(int j = 0;j <= 3;j++){
                            std::swap(l[i][j],l[swaptail][j]);
                        }
                    }
                    if(fabsl(l[i][i]) < EPS){
                        return false;
                    }
                    for(int j = 3;j >= i;j--){
                        l[i][j] /= l[i][i];
                    }
                    for(int j = i+1;j < 3;j++){
                        for(int k = 3;k >= i;k--){
                            l[j][k] -= l[j][i] * l[i][k];
                        }
                    }
                }
                for(int i = 2;i >= 0;i--){
                    for(int j = i-1;j >= 0;j--){
                        l[j][3] -= l[i][3] * l[j][i];
                    }
                }
                return true;
            };
            auto WrapPi = [&](double& val){
                while(val > M_PI)val -= 2*M_PI;
                while(val <= -M_PI)val += 2*M_PI;
            };

            double pitch = VectorToRPY(mov.GetNowPosition().ToVector()).pitch;
            double yaw = VectorToRPY(mov.GetNowPosition().ToVector()).yaw;
            double t = mov.GetNowPosition().ToVector().GetLen() / speed;
            Vector nowError = evalError(pitch,yaw,speed,t);
            if(t < minTime)return{pitch,yaw,t,nowError,AimResult::ErrorCode::TIME_ERROR};

            for(int iterationStep = 0;iterationStep < iterationTime;iterationStep++){
                Vector ep = evalError(pitch + dp,yaw,speed,t);
                Vector ey = evalError(pitch,yaw + dy,speed,t);
                Vector et = evalError(pitch,yaw,speed,t + dt);
                double matrix[3][4] = {
                    {(ep.x - nowError.x)/dp,(ey.x - nowError.x)/dy,(et.x - nowError.x)/dt,-nowError.x},
                    {(ep.y - nowError.y)/dp,(ey.y - nowError.y)/dy,(et.y - nowError.y)/dt,-nowError.y},
                    {(ep.z - nowError.z)/dp,(ey.z - nowError.z)/dy,(et.z - nowError.z)/dt,-nowError.z}
                };

                bool accepted = Solve3x3(matrix);
                if(accepted == false)break;

                double lambda = 1.0;
                accepted = false;
                for(int tryStep = 0;tryStep < tryTime;tryStep++){
                    double newPitch = pitch + lambda * matrix[0][3];
                    double newYaw   = yaw + lambda * matrix[1][3];
                    double newT     = t + lambda * matrix[2][3];
                    Vector newError = evalError(newPitch,newYaw,speed,newT);
                    if(newError.GetLen() < nowError.GetLen()){
                        pitch = newPitch,yaw = newYaw,t = newT,nowError = newError;
                        accepted = true;
                        WrapPi(yaw),WrapPi(pitch);
                        break;
                    }else{
                        lambda *= lambdaDecay;
                    }
                }
                if(accepted == false)break;
                
                if(t < minTime)return{pitch,yaw,t,nowError,AimResult::ErrorCode::TIME_ERROR};
            }
            return {pitch,yaw,t,nowError,AimResult::ErrorCode::OK};
        }
    public:
        BallisticCalculator() = delete;
        BallisticCalculator(int _iterationTime,int _tryTime,double _lambdaDecay,
            double _dp,double _dy,double _dt){
            iterationTime = _iterationTime;
            tryTime = _tryTime;
            lambdaDecay = _lambdaDecay;
            dp = _dp,dy = _dy,dt = _dt;
        }
        inline AimResult GetPitchYaw(const Movement& mov,double speed,BulletModel model = BulletModel::parabola){
            if(model == BulletModel::light){
                //特化实现
                double pitch = VectorToRPY(mov.GetNowPosition().ToVector()).pitch;
                double yaw = VectorToRPY(mov.GetNowPosition().ToVector()).yaw;
                return {pitch,yaw,0,{0,0,0},AimResult::ErrorCode::OK};
            }else if(model == BulletModel::parabola){
                auto parabolaError = [&](double _pitch,double _yaw,double _speed,double _t){
                    UniformAcceleratedMotion bullet(
                        Point(0,0,0),
                        RotateRPY(Vector(_speed,0,0),RPY(0,_pitch,_yaw)),
                        Vector(0,0,-G_VALUE)
                    );
                    return (bullet.Predict(_t) - mov.Predict(_t));
                };
                return GetPitchYawBasic(mov,speed,parabolaError);
            }
            return {0,0,0,{},AimResult::ErrorCode::MODEL_ERROR};
        }
    };
}//namespace Move3D

inline std::ostream& operator<<(std::ostream& os,const Move3D::Point& _point){
    os<<"P("<<_point.x<<','<<_point.y<<','<<_point.z<<')';
    return os;
}

inline std::ostream& operator<<(std::ostream& os,const Move3D::Vector& _vector){
    os<<"V("<<_vector.x<<','<<_vector.y<<','<<_vector.z<<')';
    return os;
}

inline std::ostream& operator<<(std::ostream& os,const Move3D::RPY& _rpy){
    os<<"RPY("<<_rpy.roll<<','<<_rpy.pitch<<','<<_rpy.yaw<<')';
    return os;
}