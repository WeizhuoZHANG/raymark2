package main;

import problem.ASVConfig;
import problem.ProblemSpec;
import tester.Tester;

import java.io.IOException;

public class Main {

    public static void main(String[] args) throws IOException{
        Tester tester = new Tester();

        String inputFileName = args[0];
        String outputFileName = args[1];
        ProblemSpec ps = new ProblemSpec();
        ps.loadProblem(inputFileName);

        ASVConfig initial = ps.getInitialState();
        ASVConfig goal = ps.getGoalState();

        ASVConfig asvConfig = sample();
    }


    public static ASVConfig sample(){

        return null;
    }
}
