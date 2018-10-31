package ca.mcgill.ecse211.project;

public interface ThreadControl {
  /**
   * run method for the Thread or Runnable object to run
   */
  public void run();
  
  /**
   * check if this poller thread is running 
   * @return: true if the thread is running, false if the thread is paused
   */
  public boolean isStarted();
  
  /**
   * start a paused thread or stop a runing thread
   * @param start
   */
  public void setStart(boolean start);
}
