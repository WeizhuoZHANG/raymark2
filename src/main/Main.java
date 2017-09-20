package main;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class Main {
    public static final double tolerance = 0.1;
    public static final double sampleNearObstacle = 0.1;
    public static final Tester tester = new Tester();
    public static final int sampleTimes = 1000;

    public static final double weight = 3.3;
    public static List<ASVConfig> ASVpath = new ArrayList<>();

    public static void main(String[] args) throws IOException{
        long startTime = System.currentTimeMillis();
        Set<ASVConfig> asvConfigs = new HashSet<ASVConfig>();

//        String inputFileName = args[0];
//        String outputFileName = args[1];

//        String inputFileName = "testcases/3ASV.txt";
        String inputFileName = "testcases/3ASV-easy.txt";
//        String inputFileName = "testcases/3ASV-x4.txt";
//        String inputFileName = "testcases/7-ASV-x2.txt";
//        String inputFileName = "testcases/7-ASV-x4.txt";
//        String inputFileName = "testcases/7-ASV-x6.txt";
//        String inputFileName = "testcases/7ASV-easy.txt";
//        String inputFileName = "testcases/7ASV.txt";
        String outputFileName = "output.txt";
        ProblemSpec ps = new ProblemSpec();
        ps.loadProblem(inputFileName);

        ASVConfig initial = ps.getInitialState();
        initial.setCost(0.0);
        ASVConfig goal = ps.getGoalState();
        int asvCount = ps.getASVCount();
//        int weight = (asvCount + ps.getObstacles().size()) / 2;
//        if (asvCount < 6){
//            weight++;
//        }else {
//            weight--;
//        }
//        if (ps.getObstacles().size() < 5){
//            weight++;
//        }else {
//            weight--;
//        }

        asvConfigs.add(initial);
        asvConfigs.add(goal);
        connect(initial, goal, asvCount, asvConfigs, ps.getObstacles());

        StringBuffer path = new StringBuffer();

        for (int i = 1; i < Math.pow(10,weight); i++) {
            sample(0, 1, 0, 1, asvCount, ps.getObstacles(), asvConfigs);
        }

        for (int i = 1; i < Math.pow(12,weight); i++) {
            for (Obstacle obstacle : ps.getObstacles()) {
                sampleObstacle(obstacle, asvCount, ps.getObstacles(), asvConfigs);
            }
        }

        if (asvCount > 5) {
            Set<ASVConfig> asvTemp = loopNeighbor(asvConfigs);
            for (int i = 0; i < 2000; i++) {
                for (ASVConfig asv : asvTemp) {
                    for (int j = 1; j < Math.pow(10, weight); j++) {
//                    sample(asv.getcspacePosition().get(0) - 0.05, asv.getcspacePosition().get(0) + 0.05, asv.getcspacePosition().get(1) - 0.1, asv.getcspacePosition().get(1) + 0.1, asvCount, ps.getObstacles(), asvConfigs);
                        sampleNoNeighbor(asv, asvCount, ps.getObstacles(), asvConfigs);
                    }
                }

                asvTemp = loopNeighbor(asvTemp);
                if (asvTemp.isEmpty())
                    break;
            }
        }

        mainLoop(initial, goal, path);

        computeMiddle(goal, ps.getObstacles());
        String result = getParentPath(goal, path);
//                + ps.calculate2(ASVpath);
        printResult(result, path);
//        printResult(path, asvConfigs);
        write(path, outputFileName);

        testNeighbor(asvConfigs);
        System.out.println("程序运行时间： " + (System.currentTimeMillis() - startTime) + "ms");
    }

    public static double mainLoop(ASVConfig initial, ASVConfig goal, StringBuffer path) {
        double shortestCost = 0;
        int count = 0;
        Comparator<ASVConfig> OrderIsdn = getComparator();
        PriorityQueue<ASVConfig> asvConfigPriorityQueue = new PriorityQueue<ASVConfig>(OrderIsdn);
        Set<ASVConfig> visited = new HashSet<ASVConfig>();

        try {
            // push initial node's start and end junction to priority queue
            asvConfigPriorityQueue.add(initial);

            // main loop to get the result
            while (!asvConfigPriorityQueue.isEmpty()) {
                ASVConfig peak = asvConfigPriorityQueue.poll();
                if (peak == goal) {
                    shortestCost = peak.getCost();
//                    count = getParentPath(peak, path);
                    break;
                }

                visited.add(peak);
                for (ASVConfig node : peak.getNeighbors().keySet()) {
                    if (!visited.contains(node)) {
                        pushSuccessor(asvConfigPriorityQueue, peak, node,
                                peak.getNeighbors().get(node));
                    }
                }
            }
        } catch (NullPointerException e) {
        }
//        printResult(count, shortestCost, path);
        return shortestCost;
    }

    public static void pushSuccessor(PriorityQueue<ASVConfig> queue, ASVConfig peak, ASVConfig node,
                                     double length) {
        // set temp cost
        double tempCost = peak.getCost() + length;
        if (node.getCost() == -1.0) {
            node.setCost(tempCost);
            queue.add(node);
            node.setParent(peak);
        } else if (tempCost < node.getCost()){
            node.setCost(tempCost);
            if (!queue.contains(node)) {
                queue.add(node);
            }
            node.setParent(peak);
        }
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
            cspace.add((Math.random() - 0.5) * 2 * Math.PI);
        }
        ASVConfig asvConfig = new ASVConfig(cspace);
        if (check(asvConfig, obstacles)){
            for (ASVConfig asvTemp : asvConfigs){
                connect(asvConfig, asvTemp,asvCount, asvConfigs, obstacles);
            }
            asvConfigs.add(asvConfig);
        }
    }


    public static void sampleNoNeighbor(ASVConfig asv, int asvCount, List<Obstacle> obstacles, Set<ASVConfig> asvConfigs){
        double delta = 0.05;
        double minX = asv.getcspacePosition().get(0) - delta;
        double minY = asv.getcspacePosition().get(1) - delta;
        List<Double> cspace = new ArrayList<Double>();
        cspace.add(minX + Math.random() * delta);
        cspace.add(minY + Math.random() * delta);
        for (int i = 2; i <= asvCount; i++){
            double theta = tester.normaliseAngle(asv.getcspacePosition().get(i) + (Math.random() - 0.5));
            cspace.add(theta);
        }
        ASVConfig asvConfig = new ASVConfig(cspace);
        if (check(asvConfig, obstacles)){
            for (ASVConfig asvTemp : asvConfigs){
                connect(asvConfig, asvTemp,asvCount, asvConfigs, obstacles);
            }
            asvConfigs.add(asvConfig);
        }
    }

    public static void connect(ASVConfig head, ASVConfig tail, int asvCount, Set<ASVConfig> asvConfigs, List<Obstacle> obstacles){
//        double maxDist = head.totalDistance(tail) / asvCount;
        double maxDist = head.maxDistance(tail);
        if (maxDist <= 0.03){
            double length = head.maxDistance(tail);
            head.addNeighbor(tail, length);
            tail.addNeighbor(head, length);
        } else if(maxDist <= tolerance){
            ASVConfig middle = new ASVConfig(middleCSpace(head, tail));
            if (check(middle, obstacles)) {
//                asvConfigs.add(middle);
                connect(middle, head,asvCount, asvConfigs, obstacles);
                connect(middle, tail,asvCount, asvConfigs, obstacles);
             }
        }
    }

//    public static boolean connect(ASVConfig head, ASVConfig tail, int asvCount, Set<ASVConfig> asvConfigs, List<Obstacle> obstacles){
////        double maxDist = head.totalDistance(tail) / asvCount;
//        double maxDist = head.maxDistance(tail);
//        if (maxDist <= 0.001){
//            double length = head.totalDistance(tail);
//            head.addNeighbor(tail, length);
//            tail.addNeighbor(head, length);
//            return true;
//        } else if (maxDist < tolerance){
//            ASVConfig middle = new ASVConfig(middleCSpace(head, tail));
//            if (check(middle, obstacles)) {
////                asvConfigs.add(middle);
//                if (connect(middle, head,asvCount, asvConfigs, obstacles) && connect(middle, tail,asvCount, asvConfigs, obstacles)){
////                    double length = head.totalDistance(tail);
////                    head.addNeighbor(tail, length);
////                    tail.addNeighbor(head, length);
//                    return true;
//                }
//            }
//        }
//        return false;
//    }

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

    public static void printResult(int count, double cost, StringBuffer path){
        if (cost == 0) {
            path.append("no-path" + System.getProperty("line.separator"));
        } else {
            path.insert(0, count+ " " + cost + System.getProperty("line.separator"));
        }
    }

    public static void printResult(String result, StringBuffer path){
        path.insert(0, result + System.getProperty("line.separator"));
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

    public static Comparator<ASVConfig> getComparator() {
        return new Comparator<ASVConfig>() {
            @Override
            public int compare(ASVConfig o1, ASVConfig o2) {
                // TODO Auto-generated method stub
                double numbera = o1.getCost();
                double numberb = o2.getCost();
                if (numbera < numberb) {
                    return -1;
                } else if (numbera > numberb) {
                    return 1;
                } else {
                    return 0;
                }
            }
        };
    }

    public static String getParentPath(ASVConfig asvConfig, StringBuffer path) {
        int count = 0;
        double cost = 0;
        ASVConfig asvConfig2 = asvConfig;
        while (asvConfig2.getParent() != null) {
            path.insert(0, asvConfig2.toString() + System.getProperty("line.separator"));
            ASVpath.add(asvConfig);
            cost += asvConfig2.totalDistance(asvConfig2.getParent());
            asvConfig2 = asvConfig2.getParent();
            count++;
        }
        ASVpath.add(asvConfig2);
        path.insert(0, asvConfig2.toString() + System.getProperty("line.separator"));
        return count++ + " " + cost;
    }

    public static Set<ASVConfig> loopNeighbor(Set<ASVConfig> asvConfigs){
        Set<ASVConfig> asvCopy = new HashSet<ASVConfig>();
        for (ASVConfig asvConfig : asvConfigs){
            if (asvConfig.getNeighbors().isEmpty()){
                asvCopy.add(asvConfig);
            }
        }
        return asvCopy;
    }

    public static void testNeighbor(Set<ASVConfig> asvConfigs){
        int i = 0;
        for (ASVConfig asvConfig : asvConfigs){
            if (asvConfig.getNeighbors().isEmpty()){
                i++;
            }
        }
        System.out.println(i);
    }

//    public static void computeMiddle(ASVConfig asvConfig, List<Obstacle> obstacles){
//        while (asvConfig.getParent() != null){
//            connectMiddle(asvConfig, asvConfig.getParent(), obstacles);
//            asvConfig = asvConfig.getParent();
//        }
//    }

//    public static boolean connectMiddle(ASVConfig head, ASVConfig tail, List<Obstacle> obstacles){
//        double maxDist = head.maxDistance(tail);
//        if (maxDist <= 0.001){
//            head.setParent(tail);
//            return true;
//        } else {
//            ASVConfig middle = new ASVConfig(middleCSpace(head, tail));
//            if (check(middle, obstacles)) {
//                connectMiddle(head, middle, obstacles);
//                connectMiddle(middle, tail, obstacles);
//            } else {
//                for (int i = 0; i < 100; i++) {
//                    ASVConfig sample = sampleSimilar(middle, obstacles);
//                    if (check(sample, obstacles)) {
//                        middle = sample;
//                        if (connectMiddle(head, middle, obstacles)) {
//                            if (connectMiddle(middle, tail, obstacles)) {
//                                return true;
//                            }
//                        }
//                    }
//                }
//            }
//            return false;
//        }
//    }

    public static void computeMiddle(ASVConfig asvConfig, List<Obstacle> obstacles){
        while (asvConfig.getParent() != null){
            ASVConfig temp = asvConfig;
            ASVConfig destination = temp.getParent();
            double count = destination.maxDistance(temp) / 0.00002;
            List<Double> cspaceDelta = new ArrayList<Double>();
            for (int i = 0; i < temp.getcspacePosition().size(); i++){
                cspaceDelta.add((temp.getcspacePosition().get(i) - destination.getcspacePosition().get(i)) / count);
            }
            for (int i = 0; i < count; i++){
                List<Double> csList = new ArrayList<Double>();
                for (int b = 0; b < cspaceDelta.size(); b++){
                    csList.add(temp.getcspacePosition().get(b) - cspaceDelta.get(b));
                }
                ASVConfig add = new ASVConfig(csList);

                for (int a = 0; a < 30000; a++){
                    if (temp.maxDistance(add) <= 0.001){
                        if (check(add, obstacles))
                            break;
                    }
                    add = sampleSimilar(temp);
                }
                if (add.maxDistance(destination) <= 0.001){
                    break;
                }
                temp.setParent(add);
                temp = add;
            }
            temp.setParent(destination);
            asvConfig = destination;
        }
    }
//    public static ASVConfig connectMiddle(ASVConfig head, ASVConfig tail, List<Obstacle> obstacles){
//        double maxDist = head.maxDistance(tail);
//        if (maxDist <= 0.001){
//            head.setParent(tail);
//            return null;
//        } else {
//            ASVConfig middle = new ASVConfig(middleCSpace(head, tail));
//            if (check(middle, obstacles)) {
//                return middle;
//            } else {
//                for (int i = 0; i < 100; i++) {
//                    ASVConfig sample = sampleSimilar(middle);
//                    if (check(sample, obstacles)) {
//                        return sample;
//                    }
//                }
//            }
//        }
//    }

    public static ASVConfig sampleSimilar(ASVConfig asv){
        double delta = 0.002;
        double minX = asv.getcspacePosition().get(0) - delta;
        double minY = asv.getcspacePosition().get(1) - delta;



        List<Double> cspace = new ArrayList<Double>();
        cspace.add(minX + Math.random() * delta);
        cspace.add(minY + Math.random() * delta);
        for (int i = 2; i <= asv.getASVCount(); i++){
            double theta = tester.normaliseAngle(asv.getcspacePosition().get(i) + (Math.random() - 0.5) * 0.01);
            cspace.add(theta);
        }
        ASVConfig asvConfig = new ASVConfig(cspace);
        return asvConfig;
    }

//    public static boolean pointCollisionObstacle(Point2D point, List<Obstacle> obs){
//        for(Obstacle obstacle: obs){
//            if(point.getX()>=obstacle.getRect().getMinX()&&point.getX()<=obstacle.getRect().getMaxX()&&point.getY()>=obstacle.getRect().getMinY()&&point.getX()<=obstacle.getRect().getMaxY()){
//                return true;
//            }
//        }
//        return false;
//
//    }
}
