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

    public static boolean isStopRecommended(Event.ForcingReactionStarted event) {
        LinkedHashMap<String, String> mapEvent = EventWriter.makeMapEvent(event); // TODO: avoid doing `makeMapEvent` here and in `write()`

        boolean isStop;
        long start = System.currentTimeMillis();
        try (Interpreter interp = new SharedInterpreter()) {
            interp.exec("import recommenderlib");
            isStop = (boolean) interp.invoke("recommenderlib.bool_", mapEvent);

            interp.exec("import events2missions");
            interp.exec("from java.lang import System");
            interp.set("map_event", mapEvent);
            interp.exec("map_event = dict(map_event)");
//            interp.exec("System.out.println(str(map_event))");
            interp.exec("df_missions = events2missions.convert_map_event_to_df_missions(map_event)");
            interp.exec("row, = df_missions.to_dict('records')");
            interp.exec("System.out.println(str(row))");
        }
        long delta = System.currentTimeMillis() - start;

        return isStop;
    }
}
