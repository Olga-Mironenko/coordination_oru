package se.oru.coordination.coordination_oru.demo;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

@Retention(RetentionPolicy.RUNTIME)

public @interface DemoDescription {

    String desc() default "N/A";

}
