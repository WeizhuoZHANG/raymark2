package main;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class Main {
    public static final double tolerance = 0.004;
    public static final double sampleNearObstacle = 0.01;
    public static final Tester tester = new Tester();
    public static final int sampleTimes = 1000;

    public static void main(String[] args) throws IOException{
        long startTime = System.currentTimeMillis();
        Set<ASVConfig> asvConfigs = new HashSet<ASVConfig>();

//        String inputFileName = args[0];
//        String outputFileName = args[1];

        String inputFileName = "testcases/7ASV.txt";
        String outputFileName = "output.txt";
        ProblemSpec ps = new ProblemSpec();
        ps.loadProblem(inputFileName);

        ASVConfig initial = ps.getInitialState();
        ASVConfig goal = ps.getGoalState();
        int asvCount = ps.getASVCount();

        asvConfigs.add(initial);
        asvConfigs.add(goal);
        connect(initial, goal, asvConfigs, ps.getObstacles());

        StringBuffer path = new StringBuffer();

        for (Obstacle obstacle : ps.getObstacles()) {
            for (int i = 1; i < 100000; i++) {
                sample(0, 1, 0, 1, asvCount, ps.getObstacles(), asvConfigs);
                sampleObstacle(obstacle, asvCount, ps.getObstacles(), asvConfigs);
            }
        }
        printResult(path, asvConfigs);
        write(path, outputFileName);

        System.out.println("程序运行时间： " + (System.currentTimeMillis() - startTime) + "ms");
    }


    public static void sampleObstacle(Obstacle obstacle, int asvCount, List<Obstacle> obstacles, Set<ASVConfig> asvConfigs){
        double minX = obstacle.getRect().getMinX();
        double maxX = obstacle.getRect().getMaxX();
        double minY = obstacle.getRect().getMinY();
        double maxY = obstacle.getRect().getMaxY();
        if (obstacle.getOrientation().get("left") == "notBounded"){
            sample(Math.min(minX - sampleNearObstacle, 0.0), minX, minY, maxY, asvCount, obstacles, asvConfigs);
        }
        if (obstacle.getOrientation().get("right") == "notBounded"){
            sample(maxX, Math.max(maxX + sampleNearObstacle, 1.0), minY, maxY, asvCount, obstacles, asvConfigs);
        }
        if (obstacle.getOrientation().get("up") == "notBounded"){
            sample(minX, maxX, maxY, Math.max(maxY + sampleNearObstacle, 1.0), asvCount, obstacles, asvConfigs);
        }
        if (obstacle.getOrientation().get("down") == "notBounded"){
            sample(minX, maxX, Math.min(minY - sampleNearObstacle, 0.0), minY, asvCount, obstacles, asvConfigs);
        }
    }

    public static void sample (double minX, double maxX, double minY, double maxY, int asvCount, List<Obstacle> obstacles, Set<ASVConfig> asvConfigs){
        List<Double> cspace = new ArrayList<Double>();
        cspace.add(minX + Math.random() * (maxX - minX));
        cspace.add(minY + Math.random() * (maxY - minY));
        for (int i = 1; i < asvCount; i++){
            cspace.add((Math.random() - 0.5) * Math.PI);
        }
        ASVConfig asvConfig = new ASVConfig(cspace);
        if (check(asvConfig, obstacles)){
            for (ASVConfig asvTemp : asvConfigs){
                connect(asvConfig, asvTemp, asvConfigs, obstacles);
            }
            asvConfigs.add(asvConfig);
        }
    }

    public static void connect(ASVConfig head, ASVConfig tail, Set<ASVConfig> asvConfigs, List<Obstacle> obstacles){
        double maxDist = head.maxDistance(tail);
        if (maxDist <= 0.001){
            double length = head.totalDistance(tail);
            head.addNeighbor(tail, length);
            tail.addNeighbor(head, length);
            return;
        } else if(maxDist <= tolerance){
            ASVConfig middle = new ASVConfig(middleCSpace(head, tail));
            if (check(middle, obstacles)) {
//                asvConfigs.add(middle);
                connect(middle, head, asvConfigs, obstacles);
                connect(middle, tail, asvConfigs, obstacles);
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

    public static boolean check(ASVConfig cfg, List<Obstacle> obstacles){
        if (!tester.isConvex(cfg)){
            return false;
        } else if (!tester.hasEnoughArea(cfg)){
            return false;
        } else if (tester.hasCollision(cfg, obstacles)){
            return false;
        } else if (!tester.fitsBounds(cfg)){
            return false;
        }
        return true;
    }

    public static void printResult(StringBuffer path, Set<ASVConfig> asvConfigs){
        String ls = System.getProperty("line.separator");
        path.append((asvConfigs.size() - 1) + " 100.0" + ls);
        for (ASVConfig asvConfig : asvConfigs){
            path.append(asvConfig.toString() + ls);
        }
    }
    public static void write(StringBuffer path, String outputfile) throws IOException{
        String ls = System.getProperty("line.separator");
        int i = path.lastIndexOf(ls);
        path = new StringBuffer(path.substring(0, i));
        FileWriter writer = new FileWriter(outputfile);
        writer.write(path.toString());
        writer.close();
    }
}
