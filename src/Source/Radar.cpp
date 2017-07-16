// Copyright 2017 Andy Taylor
#include "Radar.h"


namespace Radar {
    Point find_start(std::vector<Point> points) {
        Point s = {0, 0};
        for (int i = 0; i < points.size(); i++) {
            if ((s.x == 0 && s.y == 0)
                || (points[i].y < s.y)) {
                s = points[i];
            }
        }
        return s;
    }

    bool polarSort(Point a, Point b) {
        return a.polar < b.polar;
    }

    int checkTurn(std::vector<Point> &hull, Point p) {
        float a1 = hull[hull.size()-1].getPolar(p);
        float a2 = hull[hull.size()-2].getPolar(hull[hull.size()-1]);
        
        if (a1 > a2) {
            return 1;
        } else if (a1 == a2) {
            return 0;
        } else {
            return -1;
        }
    }

    std::vector<Point> rotate(const std::vector<Point> hull, float angle) {
        std::vector<Point> rhull;
        for (int i = 0; i < hull.size(); i++) {
            Point p = hull[i];
            rhull.push_back({static_cast<float>(p.x*cosf(angle)-p.y*sinf(angle)), static_cast<float>(p.y*cosf(angle)+p.x*sinf(angle))});
        }
        return rhull;
    }

    void calcBoundRect(std::vector<Point> &field, float *pos) {
        Point p = find_start(field);

        for (int i = 0; i < field.size(); i++) {
            field[i].polar = p.getPolar(field[i]);
        }
        
        std::sort(field.begin(), field.end(), polarSort);
        
        std::vector<Point> hull;
        hull.push_back(p);
        field.push_back(p);
        
        for (int i = 1; i < field.size(); i++) {
            while (hull.size() > 1 && checkTurn(hull, field[i]) != 1) {
                hull.pop_back();
            }
            hull.push_back(field[i]);
        }
        field.clear();
        float bestang = 0;
        float bminx, bmaxx, bminy, bmaxy;
        int bflip = 0;
        float barea = -1;
        float rectang = 0;
        for (int i = 0; i < hull.size()-1; i++) {
            float minx = 99999999999, maxx = -99999999999, miny = 99999999999, maxy = -99999999999;
            int flip = 0;
            
            rectang = hull[i].getPolar(hull[i+1]);
            if (rectang > 3.14) {
                rectang -= 3.14;
                flip = 1;
            }
            std::vector<Point> rhull = rotate(hull, -rectang);
            
            for (int l = 0; l < rhull.size(); l++) {
                minx = fmin(minx, rhull[l].x);
                maxx = fmax(maxx, rhull[l].x);
                miny = fmin(miny, rhull[l].y);
                maxy = fmax(maxy, rhull[l].y);
            }
            
            if (barea == -1 || (maxx-minx) * (maxy-miny) < barea) {
                barea = (maxx-minx) * (maxy-miny);
                bminx = minx;
                bmaxx = maxx;
                bminy = miny;
                bmaxy = maxy;
                bestang = rectang;
                bflip = flip;
            }
        }
        
        float width = (bmaxx-bminx);
        float height = (bmaxy-bminy);
        
        // Take whichever is closest to width / height
        if (width < height) {
            pos[2] = width;
            pos[3] = height;
            
            pos[4] = bestang;
            
            
            if (bestang > 2.3 || bestang < 0.8) {
                pos[0] = bmaxx;
                pos[1] = bmaxy;
            } else {
                pos[0] = -bminx;
                pos[1] = -bminy;
            }
        } else {
            pos[2] = height;
            pos[3] = width;
            
            pos[4] = bestang;
            
            pos[1] = -bminx;
            if (bestang < 0.8) {
                pos[1] = bmaxx;
            }
            if ((bestang - 1.52) > 2.3 || (bestang - 1.52) < 0.8) {
                pos[0] = -bminy;
            } else {
                pos[0] = bmaxy;
            }
        }
        
        
        // pos[2] = width;
        // pos[3] = height;
        
    }
}  // namespace Radar


