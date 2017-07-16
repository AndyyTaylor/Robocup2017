#include <math.h>

#include <vector>

namespace Radar {
    struct Point {
        int x;
        int y;
        float polar;
        
        float getPolar(Point p) {
            if (p.x == x && p.y == y) { return 0; }
            float delta_x = p.x - x;
            float delta_y = p.y - y;
            
            float ang = atan2(delta_y, delta_x);
            if (ang < 0)
                ang += 2*3.14159265;
            return ang;
        }
    };
    
    Point find_start(std::vector<Point> points);
    bool polarSort(Point a, Point b);
    int checkTurn(std::vector<Point> &hull, Point p);
    std::vector<Point> rotate(const std::vector<Point> hull, float angle);
    void calcBoundRect(std::vector<Point> &field, float *pos);
    
}  // namespace Radar
