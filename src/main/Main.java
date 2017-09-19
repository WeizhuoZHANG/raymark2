package main;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Main {
    public static final double tolerance = 0.004;

    public static void main(String[] args) throws IOException{
        Set<ASVConfig> asvConfigs = new HashSet<ASVConfig>();
        Tester tester = new Tester();

        String inputFileName = args[0];
        String outputFileName = args[1];
        ProblemSpec ps = new ProblemSpec();
        ps.loadProblem(inputFileName);

        ASVConfig initial = ps.getInitialState();
        ASVConfig goal = ps.getGoalState();

        asvConfigs.add(initial);
        asvConfigs.add(goal);
        connect(initial, goal, asvConfigs, ps.getObstacles(), tester);
        ASVConfig asvConfig = sample();

        StringBuffer path = new StringBuffer();
        write(path);
    }


    public static ASVConfig sample(){

        return null;
    }

    public static void connect(ASVConfig head, ASVConfig tail, Set<ASVConfig> asvConfigs, List<Obstacle> obstacles, Tester tester){
        double maxDist = head.maxDistance(tail);
        if (maxDist <= 0.001){
            double length = head.totalDistance(tail);
            head.addNeighbor(tail, length);
            tail.addNeighbor(head, length);
            return;
        } else if(maxDist <= tolerance){
            ASVConfig middle = new ASVConfig(middleCSpace(head, tail));
            if (check(middle, obstacles, tester)) {
                asvConfigs.add(middle);
                connect(middle, head, asvConfigs, obstacles, tester);
                connect(middle, tail, asvConfigs, obstacles, tester);
            }
        }
    }

    public static List<Double> middleCSpace(ASVConfig head, ASVConfig tail){
        List<Double> cspace = new ArrayList<Double>();
        for (int i = 0; i <= head.getASVCount(); i++){
            cspace.add((head.getcspacePosition().get(i) + tail.getcspacePosition().get(i)) / 2.0);
        }
        return cspace;
    }

    public static boolean check(ASVConfig cfg, List<Obstacle> obstacles, Tester tester){
        if (!tester.isConvex(cfg)){
            return false;
        }else if (!tester.hasEnoughArea(cfg)){
            return false;
        }
        else if (tester.hasCollision(cfg, obstacles)){
            return false;
        }
        return true;
    }

    public static void write(StringBuffer path){

    }
}
