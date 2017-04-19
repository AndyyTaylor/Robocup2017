#include "Vision.h"

#include <Arduino.h>
#include <SPI.h>
#include <Pixy.h>

namespace Camera
{
    Pixy pixy;
    struct Ball
    {
        int x, y, w, h;
    }

    #define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
    #define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

    bool init()
    {
        pixy.init();    
    }

    void update()
    {
        updateView();
    }

    void updateView()
    {
        int j;
        uint16_t blocks;
        char buf[32]; 
        int32_t panError, tiltError;

        blocks = pixy.getBlocks();

        if (blocks)
        {
            /*panError = X_CENTER-pixy.blocks[0].x;
            tiltError = pixy.blocks[0].y-Y_CENTER;

            panLoop.update(panError);
            tiltLoop.update(tiltError);

            pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
            */
            
            for (j=0; j<blocks; j++)
            {
                pixy.blocks[j].print();
                delay(100);
                if (pixy.blocks[j].signature == 0)
                {
                    
                }
            }
        }  
    }
}