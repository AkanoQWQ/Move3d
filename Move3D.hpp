namespace Move3D{
    struct Vector{
        double x,y,z;
        Vector() = default;
        Vector(double _x,double _y,double _z){
            x = _x,y = _y,z = _z;
        }
        Vector operator*(double ratio){
            return Vector(x * ratio,y * ratio,z * ratio);
        }
    };
    struct Point{
        double x,y,z;
        Point() = default;
        Point(double _x,double _y,double _z){
            x = _x,y = _y,z = _z;
        }
        Point operator+(Vector delta){
            return Point(this->x + delta.x,this->y + delta.y,this->z + delta.z);
        }
    };

    class Movement{
    protected:
        Point nowPosition;
    public:
        virtual ~Movement() = default;
        virtual Point Move(double dt) = 0;
    };

    class UniformLinearMotion : Move3D::Movement{
    private:
        Vector velocity;
    public:
        UniformLinearMotion(Point _nowPosition,Vector _velocity){
            nowPosition = _nowPosition;
            velocity = _velocity;
        }
        Move3D::Point Move(double dt) override {
            return nowPosition + velocity * dt;
        }
    };
}//namespace Move3D