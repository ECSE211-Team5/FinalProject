package ca.mcgill.ecse211.threads;

import lejos.hardware.Sound;

/**
 * class for thread control with pause and restart functionality
 *
 */
public abstract class ThreadControl implements Runnable{
  protected static int WAIT_TIME = 100;
  protected boolean isStarted;
  protected Object lockObject = new Object();
  protected boolean shouldWait;
  
  /**
   * run method implemented from Runnable class,
   * this run method implements the functionality to pause and restart the thread
   */
  public synchronized void run() {
    try {
      while (true) {
        if (!isStarted) {
          Sound.beepSequence();
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
   * check if this poller thread is running 
   * @return: true if the thread is running, false if the thread is paused
   */
  public synchronized boolean isStarted() {
    return this.isStarted;
  }
  
  /**
   * start a paused thread or stop a runing thread
   * @param start
   */
  public synchronized void setStart(boolean start) {
    isStarted = start;
    notify();
  }
  
  /**
   * wait for this thread until shouldWait is false
   */
  public void waitForThisThread() {
    synchronized(lockObject) {
      try {
        lockObject.wait();
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }
  
  /**
   * get shouldWait for this thread
   * @return: shouldWait
   */
  public boolean shouldWait() {
    synchronized(lockObject) {
      return shouldWait;
    }
  }
  
  /**
   * set if other thread shouldWait for this thread
   * @param shouldWait
   */
  public void setWait(boolean shouldWait) {
    synchronized(lockObject) {
      this.shouldWait = shouldWait;
      
      if(!shouldWait) lockObject.notifyAll();
    }
  }
  
  protected abstract void runMethod();
}
