package ca.mcgill.ecse211.threads;

/**
 * This class controls threads by providing the functionality to pause and restart them
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public abstract class ThreadControl implements Runnable {
  protected static int WAIT_TIME = 100;
  protected boolean isStarted;
  protected Object lockObject = new Object();
  protected boolean shouldWait;

  /**
   * This method implements the functionality to run a thread
   */
  public synchronized void run() {
    try {
      while (true) {
        if (!isStarted) {
          // Sound.beepSequence();
          wait();
        } else {
          runMethod();
          wait(WAIT_TIME);
        }
      }
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  /**
   * This method checks if this thread has been previously started
   * 
   * @return A boolean denoting true if the thread is running, false otherwise
   */
  public synchronized boolean isStarted() {
    return this.isStarted;
  }

  /**
   * This method starts a paused thread
   * 
   * @param start A boolean to set the current thread status
   */
  public synchronized void setStart(boolean start) {
    isStarted = start;
    notify();
  }

  /**
   * This method waits for this thread to finish
   */
  public void waitForThisThread() {
    synchronized (lockObject) {
      try {
        lockObject.wait();
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * This method returns the variable shouldWait
   * 
   * @return A boolean variable called shouldWait
   */
  public boolean shouldWait() {
    synchronized (lockObject) {
      return shouldWait;
    }
  }

  /**
   * This method sets the variable shouldWait for this thread
   * 
   * @param shouldWait A boolean variable to decide whether other threads should wait for this
   *        thread
   */
  public void setWait(boolean shouldWait) {
    synchronized (lockObject) {
      this.shouldWait = shouldWait;

      if (!shouldWait)
        lockObject.notifyAll();
    }
  }

  protected abstract void runMethod();
}
