import unittest
from cloud_queue import PendingClouds

class Clouds(unittest.TestCase):
    def test_waiting_for_exact_tf_does_not_lose_previous_frame(self):
        queue=PendingClouds()
        queue.add(1_000_000_001,'first')
        self.assertIsNone(queue.take_ready(1_200_000_000,lambda _:None))
        queue.add(1_100_000_001,'second')
        self.assertEqual(queue.take_ready(1_210_000_000,lambda m:'tf' if m=='first' else None),('first','tf'))
        self.assertEqual(queue.take_ready(1_310_000_000,lambda m:'tf'),('second','tf'))

    def test_newest_ready_avoids_accumulating_latency(self):
        queue=PendingClouds()
        for i in range(3):queue.add(1_000_000_000+i*100_000_000,i)
        self.assertEqual(queue.take_ready(1_400_000_000,lambda m:('tf',m) if m<2 else None),(1,('tf',1)))
        self.assertEqual(len(queue),1)
        self.assertFalse(queue.add(1_000_000_000,0))

    def test_original_age_limits_and_bounded_memory(self):
        queue=PendingClouds(limit=4)
        for i in range(10):queue.add(1_000_000_000+i*10_000_000,i)
        self.assertEqual(len(queue),4)
        self.assertEqual(queue.overflow_drops,6)
        self.assertIsNone(queue.take_ready(3_000_000_000,lambda m:'tf'))
        self.assertEqual(len(queue),0)
        queue.add(4_000_000_000,'future')
        self.assertIsNone(queue.take_ready(3_000_000_000,lambda m:'tf'))
        self.assertEqual(queue.stale_drops,5)

if __name__=='__main__':unittest.main()
