package main;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Point2D;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class Main {
    public static final double tolerance = 0.1;
    public static final double maxDistance = 0.001;
    public static final double sampleNearObstacle = 0.1;
    public static final Tester tester = new Tester();
    public static final int sampleTimes = 1000;
    public static final double DEFAULT_MAX_ERROR = 1e-5;

    public static final double weight = 3;
    public static List<ASVConfig> ASVpath = new ArrayList<>();

    public static HashMap<Obstacle, ArrayList<Point2D>> obstaclePoints = new HashMap<Obstacle, ArrayList<Point2D>>();

    public static void main(String[] args) throws IOException {
        long startTime = System.currentTimeMillis();
        Set<ASVConfig> asvConfigs = new HashSet<ASVConfig>();

//        String inputFileName = args[0];
//        String outputFileName = args[1];

//        String inputFileName = "testcases/3ASV.txt";
//        String inputFileName = "testcases/3ASV-easy.txt";
        String inputFileName = "testcases/3ASV-x4.txt";
//        String inputFileName = "testcases/7-ASV-x2.txt";
//        String inputFileName = "testcases/7-ASV-x4.txt";
//        String inputFileName = "testcases/7-ASV-x6.txt";
//        String inputFileName = "testcases/7ASV-easy.txt";
//        String inputFileName = "testcases/7ASV.txt";
//        String inputFileName = "testcases/01.txt";
//        String inputFileName = "testcases/02.txt";
//        String inputFileName = "testcases/03.txt";
//        String inputFileName = "testcases/04.txt";
//        String inputFileName = "testcases/05.txt";
//        String inputFileName = "testcases/06.txt";
//        String inputFileName = "testcases/07.txt";

        String outputFileName = "output.txt";
        ProblemSpec ps = new ProblemSpec();
        ps.loadProblem(inputFileName);

        ASVConfig initial = ps.getInitialState();
        initial.setCost(0.0);
        ASVConfig goal = ps.getGoalState();
        int asvCount = ps.getASVCount();

        ArrayList<Point2D> test = FindSamplePosition(ps.getObstacles(), 200);
        ArrayList<Point2D> test2 = FindSamplePositionCorner(ps.getObstacles(), 15);
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

        System.out.println("Weight: " + weight);

        asvConfigs.add(initial);
        asvConfigs.add(goal);
        connect(initial, goal, asvCount, asvConfigs, ps.getObstacles());

        StringBuffer path = new StringBuffer();

        System.out.println("loop uniform sample");
        for (int i = 1; i < Math.pow(10, weight); i++) {
            sample(0, 1, 0, 1, asvCount, ps.getObstacles(), asvConfigs);
        }

        System.out.println("loop obstacle sample");
        for (int i = 1; i < Math.pow(10, weight); i++) {
            for (Obstacle obstacle : ps.getObstacles()) {
                sampleObstacle(obstacle, asvCount, ps.getObstacles(), asvConfigs);
            }
        }

        System.out.println("loop collision free points: " + test.size());
        System.out.println("loop collision free samples");
        for (Point2D point : test) {
            for (int i = 1; i < 50; i++) {
//                sampleOnPoint(point, asvCount, ps.getObstacles(), asvConfigs);
                sampleOnPointTranslation(point, asvCount, ps.getObstacles(), asvConfigs);
            }
        }

        System.out.println("loop collision free CORNER samples");
        for (Point2D point : test2) {
            for (int i = 1; i < 5; i++) {
                sampleOnPoint(point, asvCount, ps.getObstacles(), asvConfigs);
            }
        }

        if (asvCount > 5) {
            Set<ASVConfig> asvTemp = loopNeighbor(asvConfigs);
            for (int i = 0; i < 2000; i++) {
                for (ASVConfig asv : asvTemp) {
                    for (int j = 1; j < Math.pow(10, weight - 2); j++) {
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
        printResult(result, path);
//        System.out.println("Printing Result");
//        printResult(path, asvConfigs);
        write(path, outputFileName);

//        testNeighbor(asvConfigs);
        System.out.println("程序运行时间： " + (System.currentTimeMillis() - startTime) + "ms");
    }

    public static void mainLoop(ASVConfig initial, ASVConfig goal, StringBuffer path) {
//        double shortestCost = 0;
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
//                    shortestCost = peak.getCost();
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
//        return shortestCost;
    }

    public static void pushSuccessor(PriorityQueue<ASVConfig> queue, ASVConfig peak, ASVConfig node,
                                     double length) {
        // set temp cost
        double tempCost = peak.getCost() + length;
        if (node.getCost() == -1.0) {
            node.setCost(tempCost);
            queue.add(node);
            node.setParent(peak);
        } else if (tempCost < node.getCost()) {
            node.setCost(tempCost);
            if (!queue.contains(node)) {
                queue.add(node);
            }
            node.setParent(peak);
        }
    }

    public static void sampleObstacle(Obstacle obstacle, int asvCount, List<Obstacle> obstacles, Set<ASVConfig> asvConfigs) {
        double minX = obstacle.getRect().getMinX();
        double maxX = obstacle.getRect().getMaxX();
        double minY = obstacle.getRect().getMinY();
        double maxY = obstacle.getRect().getMaxY();
        if (obstacle.getOrientation().get("left") == "notBounded") {
            sample(Math.min(minX - sampleNearObstacle, 0.0), minX, minY, maxY, asvCount, obstacles, asvConfigs);
        }
        if (obstacle.getOrientation().get("right") == "notBounded") {
            sample(maxX, Math.max(maxX + sampleNearObstacle, 1.0), minY, maxY, asvCount, obstacles, asvConfigs);
        }
        if (obstacle.getOrientation().get("up") == "notBounded") {
            sample(minX, maxX, maxY, Math.max(maxY + sampleNearObstacle, 1.0), asvCount, obstacles, asvConfigs);
        }
        if (obstacle.getOrientation().get("down") == "notBounded") {
            sample(minX, maxX, Math.min(minY - sampleNearObstacle, 0.0), minY, asvCount, obstacles, asvConfigs);
        }
    }

    public static void sample(double minX, double maxX, double minY, double maxY, int asvCount, List<Obstacle> obstacles, Set<ASVConfig> asvConfigs) {
        List<Double> cspace = new ArrayList<Double>();
        cspace.add(minX + Math.random() * (maxX - minX));
        cspace.add(minY + Math.random() * (maxY - minY));
        for (int i = 1; i < asvCount; i++) {
            cspace.add((Math.random() - 0.5) * 2 * Math.PI);
        }
        ASVConfig asvConfig = new ASVConfig(cspace);
        if (check(asvConfig, obstacles)) {
            for (ASVConfig asvTemp : asvConfigs) {
                connect(asvConfig, asvTemp, asvCount, asvConfigs, obstacles);
            }
            asvConfigs.add(asvConfig);
        }
//        checkSample(cspace, obstacles, asvConfigs, asvCount);
    }

    public static void checkSample(List<Double> cspace, List<Obstacle> obstacles, Set<ASVConfig> asvConfigs, int asvCount) {
        ASVConfig asvConfig = new ASVConfig(cspace);
        if (check(asvConfig, obstacles)) {
            int goodState = 0;
            ASVConfig nearest = asvConfig;
            double distance = 100;
            for (ASVConfig asvTemp : asvConfigs) {
                double maxDist = asvConfig.maxDistance(asvTemp);
                if (maxDist < distance) {
                    nearest = asvTemp;
                    distance = maxDist;
                }
                if (connect(asvConfig, asvTemp, asvCount, asvConfigs, obstacles)) {
                    goodState++;
                }
            }
            if (goodState == 0) {
                connectNearest(asvConfig, nearest, obstacles, distance);
            }
            asvConfigs.add(asvConfig);
        }
    }

    public static void connectNearest(ASVConfig head, ASVConfig tail, List<Obstacle> obstacles, double maxDist) {
        double length = maxDist;
        int count = (int) (maxDist / maxDistance) + 1;
        ASVConfig temp = new ASVConfig(head);
        List<Double> delta = new ArrayList<>();
        for (int i = 0; i < head.getcspacePosition().size(); i++) {
            delta.add((tail.getcspacePosition().get(i) - head.getcspacePosition().get(i)) / count);
        }
        int steps = 0;
        for (; steps < count; steps++) {
            for (int j = 0; j < temp.getcspacePosition().size(); j++) {
                temp.setCspacePosition(j, temp.getcspacePosition().get(j) + delta.get(j));
            }
            if (!check(temp, obstacles)) {
                return;
            }
        }
        head.addNeighbor(tail, length);
        tail.addNeighbor(head, length);
    }

    public static void sampleNoNeighbor(ASVConfig asv, int asvCount, List<Obstacle> obstacles, Set<ASVConfig> asvConfigs) {
        double delta = 0.05;
        double minX = asv.getcspacePosition().get(0) - delta;
        double minY = asv.getcspacePosition().get(1) - delta;
        List<Double> cspace = new ArrayList<Double>();
        cspace.add(minX + Math.random() * delta);
        cspace.add(minY + Math.random() * delta);
        for (int i = 2; i <= asvCount; i++) {
            double theta = tester.normaliseAngle(asv.getcspacePosition().get(i) + (Math.random() - 0.5));
            cspace.add(theta);
        }
        ASVConfig asvConfig = new ASVConfig(cspace);
        if (check(asvConfig, obstacles)) {
            for (ASVConfig asvTemp : asvConfigs) {
                connect(asvConfig, asvTemp, asvCount, asvConfigs, obstacles);
            }
            asvConfigs.add(asvConfig);
        }
//        checkSample(cspace, obstacles, asvConfigs, asvCount);

    }

    public static boolean connect(ASVConfig head, ASVConfig tail, int asvCount, Set<ASVConfig> asvConfigs, List<Obstacle> obstacles) {
        double aveDist = head.totalDistance(tail) / asvCount;
        double maxDist = head.maxDistance(tail);
        double length = head.maxDistance(tail);
        if (maxDist <= maxDistance) {
            head.addNeighbor(tail, length);
            tail.addNeighbor(head, length);
            return true;
        } else if (aveDist <= tolerance) {
            int count = (int) (maxDist / maxDistance) + 1;
            ASVConfig temp = new ASVConfig(head);
            List<Double> delta = new ArrayList<>();
            for (int i = 0; i < head.getcspacePosition().size(); i++) {
                delta.add((tail.getcspacePosition().get(i) - head.getcspacePosition().get(i)) / count);
            }
            int steps = 0;
            for (; steps < count; steps++) {
                for (int j = 0; j < temp.getcspacePosition().size(); j++) {
                    temp.setCspacePosition(j, temp.getcspacePosition().get(j) + delta.get(j));
                }

                if (!check(temp, obstacles)) {
//                    System.out.println("sssss");
                    return false;
                }
            }
            head.addNeighbor(tail, length);
            tail.addNeighbor(head, length);
            return true;
        }
        return false;
    }
//    public static boolean connect(ASVConfig head, ASVConfig tail, int asvCount, Set<ASVConfig> asvConfigs, List<Obstacle> obstacles){
//        double aveDist = head.totalDistance(tail) / asvCount;
//        double maxDist = head.maxDistance(tail);
//        if (maxDist <= maxDistance){
//            return true;
////            double length = head.maxDistance(tail);
////            head.addNeighbor(tail, length);
////            tail.addNeighbor(head, length);
//        } else if(aveDist <= tolerance){
//            ASVConfig middle = new ASVConfig(middleCSpace(head, tail));
//            if (check(middle, obstacles)) {
////                asvConfigs.add(middle);
//                Boolean result1 = connect(middle, head,asvCount, asvConfigs, obstacles);
//                Boolean result2 = connect(middle, tail,asvCount, asvConfigs, obstacles);
//                if (result1 && result2){
//                    double length = head.totalDistance(tail);
//                    head.addNeighbor(tail, length);
//                    tail.addNeighbor(head, length);
//                    return true;
//                }
//            }
//        }
//        return false;
//    }

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

    public static List<Double> middleCSpace(ASVConfig head, ASVConfig tail) {
        List<Double> cspace = new ArrayList<Double>();
        for (int i = 0; i <= head.getASVCount(); i++) {
            cspace.add((head.getcspacePosition().get(i) + tail.getcspacePosition().get(i)) / 2.0);
        }
        return cspace;
    }

    public static boolean check(ASVConfig cfg, List<Obstacle> obstacles) {
        if (!tester.isConvex(cfg)) {
            return false;
        } else if (!tester.hasEnoughArea(cfg)) {
            return false;
        } else if (tester.hasCollision(cfg, obstacles)) {
            return false;
        } else if (!tester.fitsBounds(cfg)) {
            return false;
        }
        return true;
    }

    public static void printResult(int count, double cost, StringBuffer path) {
        if (cost == 0) {
            path.append("no-path" + System.getProperty("line.separator"));
        } else {
            path.insert(0, count + " " + cost + System.getProperty("line.separator"));
        }
    }

    public static void printResult(String result, StringBuffer path) {
        path.insert(0, result + System.getProperty("line.separator"));
    }

    public static void printResult(StringBuffer path, Set<ASVConfig> asvConfigs) {
        String ls = System.getProperty("line.separator");
        path.append((asvConfigs.size() - 1) + " 100.0" + ls);
        for (ASVConfig asvConfig : asvConfigs) {
            path.append(asvConfig.toString() + ls);
        }
    }

    public static void write(StringBuffer path, String outputfile) throws IOException {
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

    public static Set<ASVConfig> loopNeighbor(Set<ASVConfig> asvConfigs) {
        Set<ASVConfig> asvCopy = new HashSet<ASVConfig>();
        for (ASVConfig asvConfig : asvConfigs) {
            if (asvConfig.getNeighbors().isEmpty()) {
                asvCopy.add(asvConfig);
            }
        }
        return asvCopy;
    }

    public static void testNeighbor(Set<ASVConfig> asvConfigs) {
        int i = 0;
        for (ASVConfig asvConfig : asvConfigs) {
            if (asvConfig.getNeighbors().isEmpty()) {
                i++;
            }
        }
        System.out.println(i);
    }

    public static void computeMiddle(ASVConfig asvConfig, List<Obstacle> obstacles) {
        while (asvConfig.getParent() != null) {
            ASVConfig parent = asvConfig.getParent();
            connectMiddle(asvConfig, asvConfig.getParent(), obstacles);
            asvConfig = parent;
        }
    }

    public static void connectMiddle(ASVConfig head, ASVConfig tail, List<Obstacle> obstacles) {
        double maxDist = head.maxDistance(tail);
        if (maxDist <= 0.001) {
            head.setParent(tail);
        } else {
            //           int count = (int)(maxDist / 0.0007) + 1;
            double count = maxDist / 0.0003;
            List<Double> delta = new ArrayList<>();
            ASVConfig temp = head;
            for (int i = 0; i < head.getcspacePosition().size(); i++) {
                delta.add((tail.getcspacePosition().get(i) - head.getcspacePosition().get(i)) / count);
            }
            int steps = 0;
            for (; steps < count; steps++) {
                List<Double> cspace = new ArrayList<>();
                for (int j = 0; j < temp.getcspacePosition().size(); j++) {
                    cspace.add(j, temp.getcspacePosition().get(j) + delta.get(j));
                }
                ASVConfig asvConfig = new ASVConfig(cspace);
                temp.setParent(asvConfig);
                if (temp.maxDistance(asvConfig) > 0.001) {
                    System.out.println(temp.maxDistance(asvConfig));
                }
                temp = asvConfig;
            }
            temp.setParent(tail);
        }
    }


//    public static void computeMiddle(ASVConfig asvConfig, List<Obstacle> obstacles){
//        while (asvConfig.getParent() != null){
//            ASVConfig temp = asvConfig;
//            ASVConfig destination = temp.getParent();
//            double count = destination.maxDistance(temp) / 0.00002;
//            List<Double> cspaceDelta = new ArrayList<Double>();
//            for (int i = 0; i < temp.getcspacePosition().size(); i++){
//                cspaceDelta.add((temp.getcspacePosition().get(i) - destination.getcspacePosition().get(i)) / count);
//            }
//            for (int i = 0; i < count; i++){
//                List<Double> csList = new ArrayList<Double>();
//                for (int b = 0; b < cspaceDelta.size(); b++){
//                    csList.add(temp.getcspacePosition().get(b) - cspaceDelta.get(b));
//                }
//                ASVConfig add = new ASVConfig(csList);
//
//                for (int a = 0; a < 30000; a++){
//                    if (temp.maxDistance(add) <= 0.001){
//                        if (check(add, obstacles))
//                            break;
//                    }
//                    add = sampleSimilar(temp);
//                }
//                if (add.maxDistance(destination) <= 0.001){
//                    break;
//                }
//                temp.setParent(add);
//                temp = add;
//            }
//            temp.setParent(destination);
//            asvConfig = destination;
//        }
//    }
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

    public static ASVConfig sampleSimilar(ASVConfig asv) {
        double delta = 0.0019;
        double minX = asv.getcspacePosition().get(0) - delta;
        double minY = asv.getcspacePosition().get(1) - delta;


        List<Double> cspace = new ArrayList<Double>();
        cspace.add(minX + Math.random() * delta);
        cspace.add(minY + Math.random() * delta);
        for (int i = 2; i <= asv.getASVCount(); i++) {
            double theta = tester.normaliseAngle(asv.getcspacePosition().get(i) + (Math.random() - 0.5) * 0.01);
            cspace.add(theta);
        }
        ASVConfig asvConfig = new ASVConfig(cspace);
        return asvConfig;
    }

    public static void sampleOnPoint(Point2D point, int asvCount, List<Obstacle> obstacles, Set<ASVConfig> asvConfigs) {
        List<Double> cspace = new ArrayList<Double>();
        cspace.add(point.getX());
        cspace.add(point.getY());

        for (int i = 1; i < asvCount; i++) {
            cspace.add((Math.random() - 0.5) * 2 * Math.PI);
        }
        ASVConfig asvConfig = new ASVConfig(cspace);
        if (check(asvConfig, obstacles)) {
            for (ASVConfig asvTemp : asvConfigs) {
                connect(asvConfig, asvTemp, asvCount, asvConfigs, obstacles);
            }
            asvConfigs.add(asvConfig);
        }
//        checkSample(cspace, obstacles, asvConfigs, asvCount);
    }

    public static void sampleOnPointTranslation(Point2D point, int asvCount, List<Obstacle> obstacles, Set<ASVConfig> asvConfigs) {
        List<Double> cspace = new ArrayList<Double>();
        cspace.add(point.getX());
        cspace.add(point.getY());

        for (int i = 1; i < asvCount; i++) {
            cspace.add((Math.random() - 0.5) * 2 * Math.PI);
        }
        ASVConfig asvConfig = new ASVConfig(cspace);
        if (check(asvConfig, obstacles)) {
            for (ASVConfig asvTemp : asvConfigs) {
                connect(asvConfig, asvTemp, asvCount, asvConfigs, obstacles);
            }
            asvConfigs.add(asvConfig);
            translation(asvConfig, asvConfigs, obstacles);
        }
//        checkSample(cspace, obstacles, asvConfigs, asvCount);
    }

    public static void translation(ASVConfig asvConfig, Set<ASVConfig> asvConfigs, List<Obstacle> obstacles){
        //Left translation
        double delta = 0.05;
        double threshold = 0.2;
        int asvCount = asvConfig.getASVCount();
        for (double i = delta; i < threshold;i += delta){
            List<Double> cspace = new ArrayList<>(asvConfig.getcspacePosition());
            cspace.set(0, asvConfig.getcspacePosition().get(0) + i);
            if (!checkLoop(cspace, obstacles, asvConfigs, asvCount)){
                break;
            }
        }

        //Right translation
        for (double i = delta; i < threshold;i += delta){
            List<Double> cspace = new ArrayList<>(asvConfig.getcspacePosition());
            cspace.set(0, asvConfig.getcspacePosition().get(0) - i);
            if (!checkLoop(cspace, obstacles, asvConfigs, asvCount)){
                break;
            }
        }

        //Up translation
        for (double i = delta; i < threshold;i += delta){
            List<Double> cspace = new ArrayList<>(asvConfig.getcspacePosition());
            cspace.set(0, asvConfig.getcspacePosition().get(1) + i);
            if (!checkLoop(cspace, obstacles, asvConfigs, asvCount)){
                break;
            }
        }

        //Down translation
        for (double i = delta; i < threshold;i += delta){
            List<Double> cspace = new ArrayList<>(asvConfig.getcspacePosition());
            cspace.set(0, asvConfig.getcspacePosition().get(1) - i);
            if (!checkLoop(cspace, obstacles, asvConfigs, asvCount)){
                break;
            }
        }
    }

    public static boolean checkLoop(List<Double> cspace, List<Obstacle> obstacles, Set<ASVConfig> asvConfigs, int asvCount){
        ASVConfig temp = new ASVConfig(cspace);
        if (check(temp, obstacles)) {
            for (ASVConfig asvTemp : asvConfigs) {
                connect(temp, asvTemp, asvCount, asvConfigs, obstacles);
            }
            asvConfigs.add(temp);
        } else {
            return false;
        }
        return true;
    }

    public static ArrayList<Point2D> FindSamplePosition(List<Obstacle> obstacles, double Density) {

        ArrayList<Point2D> samplePositions = new ArrayList<Point2D>();
        for (Obstacle obs : obstacles) {
            double minX = obs.getRect().getMinX();
            double minY = obs.getRect().getMinY();
            double size = obs.getRect().getWidth() * obs.getRect().getHeight();
            ArrayList<Point2D> points = new ArrayList<Point2D>();
            int sampleNO = (int) Math.round(size * Density);
            for (int i = 0; i < sampleNO; i++) {
//				double x = minX - obs.getRect().getWidth()*0.15 + (obs.getRect().getWidth() * Math.random()*0.3);
//				double y = minY - obs.getRect().getHeight()*0.15 + (obs.getRect().getHeight() * Math.random()*0.3);
                double x = minX + (obs.getRect().getWidth() * Math.random());
                double y = minY + (obs.getRect().getHeight() * Math.random());
                Point2D point = new Point2D.Double(x, y);
                points.add(point);
            }
            obstaclePoints.put(obs, points);
            //System.out.println(obstaclePoints.get(obs));
        }

        Iterator<Obstacle> obs = obstacles.iterator();

        while (obs.hasNext()) {
            Obstacle obstacleTemp = obs.next();
            ArrayList<Point2D> obsPoints = obstaclePoints.get(obstacleTemp);
            //System.out.println(obsPoints);
            for (Obstacle obs2 : obstacles) {
                if (obs2 == obstacleTemp) {
                    continue;
                }
                ArrayList<Point2D> obs2Points = obstaclePoints.get(obs2);
                for (Point2D pt : obsPoints) {
                    for (Point2D pt2 : obs2Points) {
                        double x = (pt.getX() + pt2.getX()) / 2.0;
                        double y = (pt.getY() + pt2.getY()) / 2.0;
                        Point2D point = new Point2D.Double(x, y);
                        if (pointCollisionObstacle(point, obstacles)) {
                            continue;
                        } else {
                            samplePositions.add(point);
                        }
                    }
                }
            }
            obs.remove();
        }
        return samplePositions;
    }

    public static boolean pointCollisionObstacle(Point2D point, List<Obstacle> obs) {
        for (Obstacle obstacle : obs) {
            if (point.getX() >= obstacle.getRect().getMinX() && point.getX() <= obstacle.getRect().getMaxX() && point.getY() >= obstacle.getRect().getMinY() && point.getY() <= obstacle.getRect().getMaxY()) {
                return true;
            }
        }
        return false;

    }

    //Density: 1000 - 10000
//    public static ArrayList<Point2D> FindSamplePosition(List<Obstacle> obstacles, double Density) {
//
//        ArrayList<Point2D> samplePositions = new ArrayList<Point2D>();
//        for (Obstacle obs : obstacles) {
//            double minX = obs.getRect().getMinX();
//            double minY = obs.getRect().getMinY();
//            double size = obs.getRect().getWidth() * obs.getRect().getHeight();
//            ArrayList<Point2D> points = new ArrayList<Point2D>();
//            int sampleNO = (int) Math.round(size * Density) ;
//            for (int i = 0; i < sampleNO; i++) {
//                double x = minX + (obs.getRect().getWidth() * Math.random());
//                double y = minY + (obs.getRect().getHeight() * Math.random());
//                Point2D point = new Point2D.Double(x, y);
//                points.add(point);
//            }
//            obstaclePoints.put(obs, points);
//            System.out.println(obstaclePoints.get(obs));
//        }
//
//        Iterator<Obstacle> obs = obstacles.iterator();
//
//        while (obs.hasNext()) {
//            Obstacle obstacleTemp = obs.next();
//            ArrayList<Point2D> obsPoints = obstaclePoints.get(obstacleTemp);
//            System.out.println(obsPoints);
//            for (Obstacle obs2 : obstacles) {
//                if (obs2 == obstacleTemp) {
//                    continue;
//                }
//                ArrayList<Point2D> obs2Points = obstaclePoints.get(obs2);
//                for (Point2D pt : obsPoints) {
//                    for (Point2D pt2 : obs2Points) {
//                        double x = (pt.getX() + pt2.getX()) / 2;
//                        double y = (pt.getY() + pt2.getY()) / 2;
//                        Point2D point = new Point2D.Double(x, y);
//                        if (pointCollisionObstacle(point, obstacles)) {
//                            continue;
//                        } else {
//                            samplePositions.add(point);
//                        }
//                    }
//                }
//            }
//            obs.remove();
//        }
//        return samplePositions;
//    }

    public static ArrayList<Point2D> FindSamplePositionCorner(List<Obstacle> obstacles, int sampleNO){
        ArrayList<Point2D> samplePositions = new ArrayList<Point2D>();
        for(Obstacle obs: obstacles){
            Point2D leftBottom = new Point2D.Double(obs.getRect().getMinX(), obs.getRect().getMinY());
            Point2D leftTop = new Point2D.Double(obs.getRect().getMinX(), obs.getRect().getMaxY());
            Point2D rightBottom = new Point2D.Double(obs.getRect().getMaxX(), obs.getRect().getMinY());
            Point2D rightTop = new Point2D.Double(obs.getRect().getMaxX(), obs.getRect().getMaxY());
            double width = obs.getRect().getWidth();
            double height = obs.getRect().getHeight();
            for(int i = 0;i<sampleNO; i++){
                if (leftBottom.getY() < 0.001) {
                    break;
                } else if (leftBottom.getX() < 0.001) {
                    break;
                } else {
                    double x = leftBottom.getX() - 0.05 + Math.random()*((0.2*width)+0.05);
                    double y = leftBottom.getY() - 0.05 + Math.random()*((0.2*height)+0.05);
                    Point2D pt = new Point2D.Double(x,y);
                    if(pointCollisionObstacle(pt, obstacles)){
                        continue;
                    } else {
                        samplePositions.add(pt);
                    }
                }
            }
            for(int i = 0;i<sampleNO; i++){
                if (rightBottom.getY() < 0.001) {
                    break;
                } else if (rightBottom.getX() > 0.999) {
                    break;
                } else {
                    double x = rightBottom.getX() - 0.2*width + Math.random()*((0.2*width)+0.05);
                    double y = rightBottom.getY() - 0.05 + Math.random()*((0.2*height)+0.05);
                    Point2D pt = new Point2D.Double(x,y);
                    if(pointCollisionObstacle(pt, obstacles)){
                        continue;
                    } else {
                        samplePositions.add(pt);
                    }
                }
            }
            for(int i = 0;i<sampleNO; i++){
                if (rightTop.getY() > 0.999) {
                    break;
                } else if (rightTop.getX() > 0.999) {
                    break;
                } else {
                    double x = rightTop.getX() - 0.2*width + Math.random()*((0.2*width)+0.05);
                    double y = rightTop.getY() - 0.2*height + Math.random()*((0.2*height)+0.05);
                    Point2D pt = new Point2D.Double(x,y);
                    if(pointCollisionObstacle(pt, obstacles)){
                        continue;
                    } else {
                        samplePositions.add(pt);
                    }
                }
            }
            for(int i = 0;i<sampleNO; i++){
                if (leftTop.getY() > 0.999) {
                    break;
                } else if (leftTop.getX() > 0.999) {
                    break;
                } else {
                    double x = leftTop.getX() - 0.05 + Math.random()*((0.2*width)+0.05);
                    double y = leftTop.getY() - 0.2*height + Math.random()*((0.2*height)+0.05);
                    Point2D pt = new Point2D.Double(x,y);
                    if(pointCollisionObstacle(pt, obstacles)){
                        continue;
                    } else {
                        samplePositions.add(pt);
                    }
                }
            }
        }
        return samplePositions;
    }
}
