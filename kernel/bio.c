// Buffer cache.
//
// The buffer cache is a linked list of buf structures holding
// cached copies of disk block contents.  Caching disk blocks
// in memory reduces the number of disk reads and also provides
// a synchronization point for disk blocks used by multiple processes.
//
// Interface:
// * To get a buffer for a particular disk block, call bread.
// * After changing buffer data, call bwrite to write it to disk.
// * When done with the buffer, call brelse.
// * Do not use the buffer after calling brelse.
// * Only one process at a time can use a buffer,
//     so do not keep them longer than necessary.

#include "types.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "riscv.h"
#include "defs.h"
#include "fs.h"
#include "buf.h"

#define BNUM 13 // hash table bucket number
#define HASH(x) ((uint)x % BNUM)

struct
{
  struct spinlock lock;
  struct spinlock bucket_lock[BNUM];
  struct buf buf[NBUF];
  // Linked list of all buffers, through prev/next.
  // Sorted by how recently the buffer was used.
  // head.next is most recent, head.prev is least.
  struct buf head[BNUM];
} bcache;

void binit(void)
{
  struct buf *b;
  char lockbf[8];
  for (int i = 0; i < BNUM; i++)
  {
    snprintf(lockbf, sizeof(lockbf), "bcache%d", i);
    initlock(&bcache.bucket_lock[i], lockbf);
    // Create linked list of buffers
    bcache.head[i].prev = &bcache.head[i];
    bcache.head[i].next = &bcache.head[i];
  }
  initlock(&bcache.lock, "bcache");

  for (b = bcache.buf; b < bcache.buf + NBUF; b++)
  {
    // bucket 0 is randomly selected
    b->ticks = ticks;
    b->next = bcache.head[0].next;
    b->prev = &bcache.head[0];
    initsleeplock(&b->lock, "buffer");
    bcache.head[0].next->prev = b;
    bcache.head[0].next = b;
  }
}

// Look through buffer cache for block on device dev.
// If not found, allocate a buffer.
// In either case, return locked buffer.
static struct buf *
bget(uint dev, uint blockno)
{
  struct buf *b;

  int bucketID = HASH(blockno);
  acquire(&bcache.bucket_lock[bucketID]);
  // Is the block already cached?
  for (b = bcache.head[bucketID].next; b != &bcache.head[bucketID]; b = b->next)
  {
    if (b->dev == dev && b->blockno == blockno)
    {
      b->refcnt++; // increase reference
      release(&bcache.bucket_lock[bucketID]);
      acquiresleep(&b->lock);
      return b;
    }
  }

  // Not cached.
  // Using timestamp to find the cache rarely used
  // We need to check all buckets
  for (int i = (bucketID + 1) % BNUM; i != bucketID; i = (i + 1) % BNUM)
  {
    struct buf *rbuf = 0;
    uint rticks = 0xFFFFFFFF;
    acquire(&bcache.bucket_lock[i]);
    for (b = bcache.head[i].prev; b != &bcache.head[i]; b = b->prev)
    {
      if (b->refcnt == 0 && b->ticks < rticks)
      {
        rticks = b->ticks;
        rbuf = b;
      } // A free and rarely used cache

      if (rbuf != 0)
      {
        rbuf->dev = dev;
        rbuf->blockno = blockno;
        rbuf->valid = 0;
        rbuf->refcnt = 1;
        rbuf->ticks = ticks; // new cache
        // no one is waiting for it.
        // copy from original brelse() function
        rbuf->next->prev = rbuf->prev;
        rbuf->prev->next = rbuf->next;
        rbuf->next = bcache.head[bucketID].next;
        rbuf->prev = &bcache.head[bucketID];
        bcache.head[bucketID].next->prev = rbuf;
        bcache.head[bucketID].next = rbuf;
        // release in order
        release(&bcache.bucket_lock[i]);
        release(&bcache.bucket_lock[bucketID]);
        acquiresleep(&rbuf->lock);
        return rbuf;
      }
    }
    release(&bcache.bucket_lock[i]);
  }
  panic("bget: no buffers");
}

// Return a locked buf with the contents of the indicated block.
struct buf *
bread(uint dev, uint blockno)
{
  struct buf *b;

  b = bget(dev, blockno);
  if (!b->valid)
  {
    virtio_disk_rw(b, 0);
    b->valid = 1;
  }
  return b;
}

// Write b's contents to disk.  Must be locked.
void bwrite(struct buf *b)
{
  if (!holdingsleep(&b->lock))
    panic("bwrite");
  virtio_disk_rw(b, 1);
}

// Release a locked buffer.
// Move to the head of the most-recently-used list.
void brelse(struct buf *b)
{
  if (!holdingsleep(&b->lock))
    panic("brelse");

  releasesleep(&b->lock);

  uint bucketID = HASH(b->blockno);
  acquire(&bcache.bucket_lock[bucketID]);
  b->refcnt--;
  if (b->refcnt == 0)
  {
    // update cache timestamp only when the cache last used
    b->ticks = ticks;
  }
  release(&bcache.bucket_lock[bucketID]);
}

void bpin(struct buf *b)
{
  uint bucketID = HASH(b->blockno);
  acquire(&bcache.bucket_lock[bucketID]);
  b->refcnt++;
  release(&bcache.bucket_lock[bucketID]);
}

void bunpin(struct buf *b)
{
  uint bucketID = HASH(b->blockno);
  acquire(&bcache.bucket_lock[bucketID]);
  b->refcnt--;
  release(&bcache.bucket_lock[bucketID]);
}