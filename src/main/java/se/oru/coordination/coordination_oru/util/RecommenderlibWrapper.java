package se.oru.coordination.coordination_oru.util;

import jep.Interpreter;
import jep.SharedInterpreter;

import java.util.LinkedHashMap;

public class RecommenderlibWrapper {
    static {
        try (Interpreter interp = new SharedInterpreter()) {
            interp.exec("import sys, os");
            interp.exec("sys.path = ['', '/home/olga/miniconda3/lib/python312.zip', '/home/olga/miniconda3/lib/python3.12', '/home/olga/miniconda3/lib/python3.12/lib-dynload', '/home/olga/miniconda3/lib/python3.12/site-packages']");
            interp.exec("sys.path.append(os.getcwd() + '/scenario-analysis')");

            interp.exec("import recommenderlib");
            interp.exec("recommenderlib.bool_ = bool");
        }
    }

    public static boolean isStopRecommended(int robotID) {
//        LinkedHashMap<String, String> mapEvent = EventWriter.makeMapEvent(event);

        boolean isStop;
        long start = System.currentTimeMillis();
        try (Interpreter interp = new SharedInterpreter()) {
            interp.exec("import recommenderlib");
            isStop = (boolean) interp.invoke("recommenderlib.bool_", robotID);
        }
        long delta = System.currentTimeMillis() - start;

        return isStop;
    }
}
