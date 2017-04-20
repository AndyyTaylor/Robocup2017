#include "Vision.h"

#include <Arduino.h>
#include <SPI.h>
#include <Pixy.h>

namespace Vision
{
    class ServoLoop
    {
    public:
      ServoLoop(int32_t pgain, int32_t dgain);

      void update(int32_t error);
       
      int32_t m_pos;
      int32_t m_prevError;
      int32_t m_pgain;
      int32_t m_dgain;
    };


    ServoLoop panLoop(300, 500);
    ServoLoop tiltLoop(500, 700);

    ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
    {
      m_pos = PIXY_RCS_CENTER_POS;
      m_pgain = pgain;
      m_dgain = dgain;
      m_prevError = 0x80000000L;
    }

    void ServoLoop::update(int32_t error)
    {
      long int vel;
      char buf[32];
      if (m_prevError!=0x80000000)
      {	
        vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>10;
        //sprintf(buf, "%ld\n", vel);
        //Serial.print(buf);
        m_pos += vel;
        if (m_pos>PIXY_RCS_MAX_POS) 
          m_pos = PIXY_RCS_MAX_POS; 
        else if (m_pos<PIXY_RCS_MIN_POS) 
          m_pos = PIXY_RCS_MIN_POS;
      }
      m_prevError = error;
    }

    struct Ball
    {
        int x, y, w, h;
        float angle;
        
        void setAngle(){
            angle = (((float) x/319.0f) - 0.5) * 75;
        }
        
        float getAngle(){
            return angle;
        }
    };

    #define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)
    #define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)
    
    Pixy pixy;
    Ball ball;

    bool init()
    {
        pixy.init();
        return true;
    }

    void update()
    {
        updateView();
    }

    void updateView()
    {
        uint16_t blocks;
        char buf[32];
        //Serial.println(ball.x);
        blocks = pixy.getBlocks();

        if (blocks)
        {
            
            for (int i=0; i<blocks; i++)
            {
                //Serial.println(pixy.blocks[i].signature);
                if (pixy.blocks[i].signature == 1)
                {
                    ball.x = pixy.blocks[i].x;
                    ball.y = pixy.blocks[i].y;
                    ball.w = pixy.blocks[i].width;
                    ball.h = pixy.blocks[i].height;
                    ball.setAngle();
                    //lookAtBall();
                }
            }
        }
    }
    
    void lookAtBall()
    {
        int32_t panError, tiltError;
        panError = X_CENTER-pixy.blocks[0].x;
        tiltError = pixy.blocks[0].y-Y_CENTER;

        panLoop.update(panError);
        tiltLoop.update(tiltError);

        pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
    }
    
    float getBallAngle()
    {
        return ball.getAngle();
    }
}
