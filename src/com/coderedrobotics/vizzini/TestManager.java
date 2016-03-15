package com.coderedrobotics.vizzini;

import java.util.ArrayList;

import com.coderedrobotics.libs.Logger;
import com.coderedrobotics.libs.Logger.Level;

public class TestManager {

    private ArrayList<TestStage> stages = new ArrayList<>();

    private int currentStage;

    public void addStage(TestCommand command, TestFinalize finalize,
            int timeout, TestResult timeoutResult, String subsystemName) {
        stages.add(new TestStage(command, finalize, timeout, timeoutResult, subsystemName));
    }

    public void step() {
        if (isComplete()) {
            return;
        }
        TestStage stage = stages.get(currentStage);
        stage.step();
        if (stage.getStatus() != TestResult.INCONCLUSIVE) {
            Logger.getInstance().log(Level.INFO, 304, stage.getResultMessage());
            currentStage++;
        }
        if (isComplete()) {
            int suceededTests = 0;
            for (TestStage testStage : stages) {
                suceededTests += testStage.getStatus() == TestResult.SUCESS ? 1 : 0;
            }
            Logger.getInstance().log(Level.INFO, 304, "self test complete. " + suceededTests + "/" + currentStage + " tests passed.");
        }
    }

    public void reset() {
        for (TestStage testStage : stages) {
            testStage.reset();
        }
        currentStage = 0;
    }

    public boolean isComplete() {
        return currentStage == stages.size();
    }

    private class TestStage {

        private final TestCommand command;
        private final TestFinalize finalize;
        private final int timeout;
        private final TestResult timeoutResult;
        private final String name;

        private TestResult result;
        private long endTime = -1;

        private TestStage(TestCommand command, TestFinalize finalize,
                int timeout, TestResult timeoutResult, String subsystemName) {
            if (timeoutResult == TestResult.INCONCLUSIVE) {
                throw new IllegalArgumentException("cannot have inconclusive timeout");
            }
            if (subsystemName == null) {
                throw new IllegalArgumentException("subsystemName is Null");
            }

            this.command = command;
            this.finalize = finalize;
            this.timeout = timeout;
            this.timeoutResult = timeoutResult;
            this.name = subsystemName;

            result = TestResult.INCONCLUSIVE;
        }

        private void reset() {
            result = TestResult.INCONCLUSIVE;
            endTime = -1;
        }

        private void step() {
            if (result != TestResult.INCONCLUSIVE) {
                throw new IllegalStateException();
            }
            if (endTime == -1) {
                endTime = System.currentTimeMillis() + timeout;
            }
            result = command.RunTest();
            if (System.currentTimeMillis() > endTime) {
                result = timeoutResult;
            }
            if (result != TestResult.INCONCLUSIVE) {
                finalize.FinalizeTest();
            }
        }

        private TestResult getStatus() {
            return result;
        }

        private String getResultMessage() {
            if (result == TestResult.INCONCLUSIVE) {
                throw new IllegalStateException();
            }
            return name + " test " + (result == TestResult.SUCESS ? "succeeded" : "failed");
        }
    }

    public interface TestCommand {

        public TestResult RunTest();
    }

    public interface TestFinalize {

        public void FinalizeTest();
    }
}
