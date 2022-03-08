package org.firstinspires.ftc.teamcode.opmode.auton;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Slides;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class AutonSlides {

    ScheduledExecutorService service;
    Slides slides;

    public Runnable score3, dump3, retract;
    public Runnable score2, slides2, dump2, retract2;
    public Runnable score1, slides1, dump1, retract1;

    public AutonSlides(Slides slidesObj, ScheduledExecutorService serviceObj){


        this.service = serviceObj;
        this.slides = slidesObj;

        //Level3 Code
        retract = new Runnable() {
            @Override
            public void run() {
                slides.retract();
                slides.resetServos();
            }
        };

        dump3 = new Runnable() {
            @Override
            public void run() {
                slides.dump3();

                service.schedule(retract, (long) Slides.SERVO_DELAY_TIME * 1000, TimeUnit.MILLISECONDS);
            }
        };

       score3 = new Runnable() {
            @Override
            public void run() {
                slides.extend();

                service.schedule(dump3, (long) (Slides.SLIDES_DELAY_TIME * 1000), TimeUnit.MILLISECONDS);
            }
        };

        // Level 2 Code
        retract2 = new Runnable() {
            @Override
            public void run() {
                slides.retract2();
                slides.resetServos();
            }
        };

        dump2 = new Runnable() {
            @Override
            public void run() {
                slides.dump2();

                service.schedule(retract2, (long) Slides.SERVO_DELAY_TIME * 1000, TimeUnit.MILLISECONDS);
            }
        };

        slides2 = new Runnable() {
            @Override
            public void run() {
                slides.level2Extend();

                service.schedule(dump2, (long) (Slides.SLIDES_DELAY_TIME * 1000), TimeUnit.MILLISECONDS);
            }
        };

        score2 = new Runnable() {
            @Override
            public void run() {
                slides.temp2();

                service.schedule(slides2, (long) (Slides.SLIDES_DELAY_TIME * 1000), TimeUnit.MILLISECONDS);
            }
        };

        //Level 1 Code
        retract1 = new Runnable() {
            @Override
            public void run() {
                slides.retract1();
                slides.resetServos();
            }
        };

        dump1 = new Runnable() {
            @Override
            public void run() {
                slides.dump1();

                service.schedule(retract1, (long) Slides.SERVO_DELAY_TIME * 1000, TimeUnit.MILLISECONDS);
            }
        };

        slides1 = new Runnable() {
            @Override
            public void run() {
                slides.level2Extend();

                service.schedule(dump1, (long) (Slides.SLIDES_DELAY_TIME * 1000), TimeUnit.MILLISECONDS);
            }
        };

        score1 = new Runnable() {
            @Override
            public void run() {
                slides.temp1();

                service.schedule(slides1, (long) (Slides.SLIDES_DELAY_TIME * 1000), TimeUnit.MILLISECONDS);
            }
        };


    }

    public void level3(){
        service.submit(score3);
    }

    public void level2(){
        service.submit(score2);
    }

    public void level1(){
        service.submit(score1);
    }
}
