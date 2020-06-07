/*
 * TCP CUBIC: Binary Increase Congestion control for TCP v2.3
 * Home page:
 *      http://netsrv.csc.ncsu.edu/twiki/bin/view/Main/BIC
 * This is from the implementation of CUBIC TCP in
 * Sangtae Ha, Injong Rhee and Lisong Xu,
 *  "CUBIC: A New TCP-Friendly High-Speed TCP Variant"
 *  in ACM SIGOPS Operating System Review, July 2008.
 * Available from:
 *  http://netsrv.csc.ncsu.edu/export/cubic_a_new_tcp_2008.pdf
 *
 * CUBIC integrates a new slow start algorithm, called HyStart.
 * The details of HyStart are presented in
 *  Sangtae Ha and Injong Rhee,
 *  "Taming the Elephants: New TCP Slow Start", NCSU TechReport 2008.
 * Available from:
 *  http://netsrv.csc.ncsu.edu/export/hystart_techreport_2008.pdf
 *
 * All testing results are available from:
 * http://netsrv.csc.ncsu.edu/wiki/index.php/TCP_Testing
 *
 * Unless CUBIC is enabled and congestion window is large
 * this behaves the same as the original Reno.
 */
 
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <net/tcp.h>
 
#define BICTCP_BETA_SCALE    1024	/* Scale factor beta calculation
					 * max_cwnd = snd_cwnd * beta
					 */
#define	BICTCP_HZ		10	/* BIC HZ 2^10 = 1024 */
 
/* Two methods of hybrid slow start */
#define HYSTART_ACK_TRAIN	0x1
#define HYSTART_DELAY		0x2
 
/* Number of delay samples for detecting the increase of delay */
#define HYSTART_MIN_SAMPLES	8
#define HYSTART_DELAY_MIN	(2U<<3)
#define HYSTART_DELAY_MAX	(16U<<3)
#define HYSTART_DELAY_THRESH(x)	clamp(x, HYSTART_DELAY_MIN, HYSTART_DELAY_MAX)
static int fast_convergence __read_mostly = 1;
static int beta __read_mostly = 717;	/* = 717/1024 (BICTCP_BETA_SCALE) */
static int initial_ssthresh __read_mostly;
//bic_scale就是paper中三次方系数C的1024倍缩放值
//MODULE_PARM_DESC(bic_scale, "scale (scaled by 1024) value for bic function (bic_scale/1024)");
static int bic_scale __read_mostly = 41;
static int tcp_friendliness __read_mostly = 1;
static int hystart __read_mostly = 1;
static int hystart_detect __read_mostly = HYSTART_ACK_TRAIN | HYSTART_DELAY;
static int hystart_low_window __read_mostly = 16;
static u32 cube_rtt_scale __read_mostly;
static u32 beta_scale __read_mostly;
static u64 cube_factor __read_mostly;
/* Note parameters that are used for precomputing scale factors are read-only */
module_param(fast_convergence, int, 0644);
MODULE_PARM_DESC(fast_convergence, "turn on/off fast convergence");
module_param(beta, int, 0644);
MODULE_PARM_DESC(beta, "beta for multiplicative increase");
module_param(initial_ssthresh, int, 0644);
MODULE_PARM_DESC(initial_ssthresh, "initial value of slow start threshold");
module_param(bic_scale, int, 0444);
MODULE_PARM_DESC(bic_scale, "scale (scaled by 1024) value for bic function (bic_scale/1024)");
module_param(tcp_friendliness, int, 0644);
MODULE_PARM_DESC(tcp_friendliness, "turn on/off tcp friendliness");
module_param(hystart, int, 0644);
MODULE_PARM_DESC(hystart, "turn on/off hybrid slow start algorithm");
module_param(hystart_detect, int, 0644);
MODULE_PARM_DESC(hystart_detect, "hyrbrid slow start detection mechanisms"
		 " 1: packet-train 2: delay 3: both packet-train and delay");
module_param(hystart_low_window, int, 0644);
MODULE_PARM_DESC(hystart_low_window, "lower bound cwnd for hybrid slow start");
/* BIC TCP Parameters */
struct bictcp {
	u32	cnt;		/* increase cwnd by 1 after ACKs */
	u32 	last_max_cwnd;	/* last maximum snd_cwnd */
//两个重要的count值:
//第一个是tcp_sock->snd_cwnd_cnt，表示在当前的拥塞窗口中已经
//发送(经过对方ack包确认)的数据段的个数，
//而第二个是bictcp->cnt，它是cubic拥塞算法的核心，
//主要用来控制在拥塞避免状态的时候，什么时候才能增大拥塞窗口，
//具体实现是通过比较cnt和snd_cwnd_cnt，来决定是否增大拥塞窗口，
	u32	loss_cwnd;	/* congestion window at last loss */
	u32	last_cwnd;	/* the last snd_cwnd */
	u32	last_time;	/* time when updated last_cwnd */
	u32	bic_origin_point;/* origin point of bic function */
	u32	bic_K;		/* time to origin point from the beginning of the current epoch */
	u32	delay_min;	/* min delay */
	u32	epoch_start;	/* beginning of an epoch */
	u32	ack_cnt;	/* number of acks */
	u32	tcp_cwnd;	/* estimated tcp cwnd */
#define ACK_RATIO_SHIFT	4
	u16	delayed_ack;	/* estimate the ratio of Packets/ACKs << 4 */
	u8	sample_cnt;	/* number of samples to decide curr_rtt */
	u8	found;		/* the exit point is found? */
	u32	round_start;	/* beginning of each round */
	u32	end_seq;	/* end_seq of the round */
	u32	last_jiffies;	/* last time when the ACK spacing is close */
	u32	curr_rtt;	/* the minimum rtt of current round */
};
static inline void bictcp_reset(struct bictcp *ca)
{
	ca->cnt = 0;
	ca->last_max_cwnd = 0;
	ca->loss_cwnd = 0;
	ca->last_cwnd = 0;
	ca->last_time = 0;
	ca->bic_origin_point = 0;
	ca->bic_K = 0;
	ca->delay_min = 0;
	ca->epoch_start = 0;
	ca->delayed_ack = 2 << ACK_RATIO_SHIFT;
	ca->ack_cnt = 0;
	ca->tcp_cwnd = 0;
	ca->found = 0;
}
 
static inline void bictcp_hystart_reset(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bictcp *ca = inet_csk_ca(sk);
 
	ca->round_start = ca->last_jiffies = jiffies;
	ca->end_seq = tp->snd_nxt;
	ca->curr_rtt = 0;
	ca->sample_cnt = 0;
}
 
static void bictcp_init(struct sock *sk)
{
	bictcp_reset(inet_csk_ca(sk));
 
	if (hystart)
		bictcp_hystart_reset(sk);
 
	if (!hystart && initial_ssthresh)
		tcp_sk(sk)->snd_ssthresh = initial_ssthresh;
}
 
/* calculate the cubic root of x using a table lookup followed by one
 * Newton-Raphson iteration.
 * Avg err ~= 0.195%
 */
static u32 cubic_root(u64 a)
{
	u32 x, b, shift;
	/*
	 * cbrt(x) MSB values for x MSB values in [0..63].
	 * Precomputed then refined by hand - Willy Tarreau
	 *
	 * For x in [0..63],
	 *   v = cbrt(x << 18) - 1
	 *   cbrt(x) = (v[x] + 10) >> 6
	 */
	static const u8 v[] = {
		/* 0x00 */    0,   54,   54,   54,  118,  118,  118,  118,
		/* 0x08 */  123,  129,  134,  138,  143,  147,  151,  156,
		/* 0x10 */  157,  161,  164,  168,  170,  173,  176,  179,
		/* 0x18 */  181,  185,  187,  190,  192,  194,  197,  199,
		/* 0x20 */  200,  202,  204,  206,  209,  211,  213,  215,
		/* 0x28 */  217,  219,  221,  222,  224,  225,  227,  229,
		/* 0x30 */  231,  232,  234,  236,  237,  239,  240,  242,
		/* 0x38 */  244,  245,  246,  248,  250,  251,  252,  254,
	};
 
	b = fls64(a);
	if (b < 7) {
		/* a in [0..63] */
		return ((u32)v[(u32)a] + 35) >> 6;
	}
 
	b = ((b * 84) >> 8) - 1;
	shift = (a >> (b * 3));
 
	x = ((u32)(((u32)v[shift] + 10) << b)) >> 6;
 
	/*
	 * Newton-Raphson iteration
	 *                         2
	 * x    = ( 2 * x  +  a / x  ) / 3
	 *  k+1          k         k
	 */
	x = (2 * x + (u32)div64_u64(a, (u64)x * (u64)(x - 1)));
	x = ((x * 341) >> 10);
	return x;
}
/*
函数关键点
1.  我们最终要得到的是ca->cnt，用来控制snd_cwnd的增长。
2.  ca->cnt的值，是根据cwnd和w( t + after ) 的大小来判断的。w( t + after )即bic_target，它表示我们预期的
在经过after时间后的snd_cwnd。如果此时cwnd < w( t + after )，那么我们就快速增加窗口，达到预期目标。
如果cwnd > w( t + after )，那说明我们已经增加过快了，需要降速了，这样才能达到预期目标。
              cwnd / (bic_target - cwnd )   // bic_target > cwnd 
cnt =      
             100 * cwnd   // bic_target < cwnd 
3.  cwnd是传入的参数，已知。现在我们只需要计算bic_target。
而根据Cubic的窗口增长函数：W(t) = C(t - K)^3 + Wmax，
我们要计算时间( 当前 + after )，以及时间K。时间K即bic_K，表示函数值为Wmax所对应的时间。
通过代码可以发现，after为min RTT，即连接的传播时延。
*/
/*
 * Compute congestion window to use.
 */
static inline void bictcp_update(struct bictcp *ca, u32 cwnd)
{
	u64 offs;/* 时间差，| t - K | */  
	u32 delta, t, bic_target, max_cnt;/* delta是cwnd差，bic_target是预测值，t为预测时间 */  
	ca->ack_cnt++;	/* count the number of ACKs */
 
	if (ca->last_cwnd == cwnd &&
	    (s32)(tcp_time_stamp - ca->last_time) <= HZ / 32)
		return;
 
	ca->last_cwnd = cwnd;
	ca->last_time = tcp_time_stamp;
 
	if (ca->epoch_start == 0) {
		ca->epoch_start = tcp_time_stamp;	/* record the beginning of an epoch */
		ca->ack_cnt = 1;			/* start counting */
		ca->tcp_cwnd = cwnd;			/* syn with cubic */
                /* 取max(last_max_cwnd , cwnd)作为当前Wmax */  
		if (ca->last_max_cwnd <= cwnd) {
			ca->bic_K = 0;
			ca->bic_origin_point = cwnd;
		} else {
			/* Compute new K based on
			 * (wmax-cwnd) * (srtt>>3 / HZ) / c * 2^(3*bictcp_HZ)
			 */
			ca->bic_K = cubic_root(cube_factor
					       * (ca->last_max_cwnd - cwnd));
			ca->bic_origin_point = ca->last_max_cwnd;
		}
	}
 
	/* cubic function - calc*/
	/* calculate c * time^3 / rtt,
	 *  while considering overflow in calculation of time^3
	 * (so time^3 is done by using 64 bit)
	 * and without the support of division of 64bit numbers
	 * (so all divisions are done by using 32 bit)
	 *  also NOTE the unit of those veriables
	 *	  time  = (t - K) / 2^bictcp_HZ
	 *	  c = bic_scale >> 10
	 * rtt  = (srtt >> 3) / HZ
	 * !!! The following code does not have overflow problems,
	 * if the cwnd < 1 million packets !!!
	 */
	/* change the unit from HZ to bictcp_HZ */
	t = ((tcp_time_stamp + (ca->delay_min>>3) - ca->epoch_start)
	     << BICTCP_HZ) / HZ;
        /* 求| t - bic_K | */  
	if (t < ca->bic_K)	/* 还未达到Wmax */  
		offs = ca->bic_K - t;
	else
		offs = t - ca->bic_K; //此时已经超过Wmax
 
	/* c/rtt * (t-K)^3 */
        /* 计算delta =| W(t) - W(bic_K) |  
         * cube_rtt_scale = (bic_scale * 10) = c / srtt * 2^10，c/srtt = 0.4 
         */
	delta = (cube_rtt_scale * offs * offs * offs) >> (10+3*BICTCP_HZ);
	if (t < ca->bic_K)                                	/* below origin*/
		bic_target = ca->bic_origin_point - delta;
	else                                                	/* above origin*/
		bic_target = ca->bic_origin_point + delta;
 
	/* cubic function - calc bictcp_cnt /* 计算bic_target，即预测cwnd */
	if (bic_target > cwnd) {
                /* 相差越多，增长越快，这就是函数形状由来 */
		ca->cnt = cwnd / (bic_target - cwnd);
	} else {
		ca->cnt = 100 * cwnd;              /* very small increment 目前cwnd已经超出预期了，应该降速*/
	}
 
	/* TCP Friendly  /* TCP Friendly —如果cubic比RENO慢，则提升cwnd增长速度，即减小cnt 
         * 以上次丢包以后的时间t算起，每次RTT增长 3B / ( 2 - B)，那么可以得到 
          * 采用RENO算法的cwnd。 
          * cwnd (RENO) = cwnd + 3B / (2 - B) * ack_cnt / cwnd 
         * B为乘性减少因子，在此算法中为0.3 
         */
	if (tcp_friendliness) {
		u32 scale = beta_scale;
		delta = (cwnd * scale) >> 3;
		while (ca->ack_cnt > delta) {		/* update tcp cwnd */
			ca->ack_cnt -= delta;
			ca->tcp_cwnd++;
		}
 
		if (ca->tcp_cwnd > cwnd){	/* if bic is slower than tcp */
			delta = ca->tcp_cwnd - cwnd;
			max_cnt = cwnd / delta;
			if (ca->cnt > max_cnt)
				ca->cnt = max_cnt;
		}
	}
 
	ca->cnt = (ca->cnt << ACK_RATIO_SHIFT) / ca->delayed_ack;//做了一个比较直接的delay_ack的控制
        //ratio = (15*ratio + sample) / 16 这里ca-delayed_ack由
	if (ca->cnt == 0)			/* cannot be zero */
		ca->cnt = 1;/* 此时代表cwnd远小于bic_target，增长速度最大 */  
}
 
static void bictcp_cong_avoid(struct sock *sk, u32 ack, u32 in_flight)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bictcp *ca = inet_csk_ca(sk);
        //判断发送拥塞窗口是否到达限制，如果到达限制则直接返回
	if (!tcp_is_cwnd_limited(sk, in_flight))
		return;
 
	if (tp->snd_cwnd <= tp->snd_ssthresh) {
		if (hystart && after(ack, ca->end_seq))
			bictcp_hystart_reset(sk);
		tcp_slow_start(tp);//进入slow start状态
	} else {
		bictcp_update(ca, tp->snd_cwnd);
		tcp_cong_avoid_ai(tp, ca->cnt);//然后进入拥塞避免，更新tcp_sock->snd_cwnd_cnt
	}
 
}
 
//做了两件事：重赋值last_max_cwnd、返回新的慢启动阈值
static u32 bictcp_recalc_ssthresh(struct sock *sk)
{
	const struct tcp_sock *tp = tcp_sk(sk);
	struct bictcp *ca = inet_csk_ca(sk);
 
	ca->epoch_start = 0;	/* end of epoch */
 
	/* Wmax and fast convergence */
        //当一个新的TCP流加入到网络，
        //网络中已有TCP流需要放弃自己带宽，
        //给新的TCP流提供一定的上升空间。
        //为提高已有TCP流所释放的带宽而引入快速收敛机制。
	if (tp->snd_cwnd < ca->last_max_cwnd && fast_convergence)
        //表示已有TCP流所经历的饱和点因为可用带宽改变而正在降低。
        //然后，通过进一步降低Wmax让已有流释放更多带宽。
        //这种行为有效地延长已有流增大其窗口的时间，
        //因为降低后的Wmax强制已有流更早进入平稳状态。
        //这允许新流有更多的时间来赶上其窗口尺寸。
		ca->last_max_cwnd = (tp->snd_cwnd * (BICTCP_BETA_SCALE + beta))
			/ (2 * BICTCP_BETA_SCALE);//last_max_cwnd = 0.9 * snd_cwnd
	else
		ca->last_max_cwnd = tp->snd_cwnd;
 
	ca->loss_cwnd = tp->snd_cwnd;
        //修改snd_ssthresh，即max(0.7*snd_cwnd，2)
	return max((tp->snd_cwnd * beta) / BICTCP_BETA_SCALE, 2U);
}
 
//快速恢复：直接把snd_cwnd更新为max(snd_cwnd，last_max_cwnd)，和掉包前相差不大
static u32 bictcp_undo_cwnd(struct sock *sk)
{
	struct bictcp *ca = inet_csk_ca(sk);
 
	return max(tcp_sk(sk)->snd_cwnd, ca->last_max_cwnd);
}
 
static void bictcp_state(struct sock *sk, u8 new_state)
{
	if (new_state == TCP_CA_Loss) {
		bictcp_reset(inet_csk_ca(sk));
		bictcp_hystart_reset(sk);
	}
}
 
static void hystart_update(struct sock *sk, u32 delay)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bictcp *ca = inet_csk_ca(sk);
 
	if (!(ca->found & hystart_detect)) {
		u32 curr_jiffies = jiffies;
 
		/* first detection parameter - ack-train detection */
		if (curr_jiffies - ca->last_jiffies <= msecs_to_jiffies(2)) {
			ca->last_jiffies = curr_jiffies;
			if (curr_jiffies - ca->round_start >= ca->delay_min>>4)
				ca->found |= HYSTART_ACK_TRAIN;
		}
 
		/* obtain the minimum delay of more than sampling packets */
		if (ca->sample_cnt < HYSTART_MIN_SAMPLES) {
			if (ca->curr_rtt == 0 || ca->curr_rtt > delay)
				ca->curr_rtt = delay;
 
			ca->sample_cnt++;
		} else {
			if (ca->curr_rtt > ca->delay_min +
			    HYSTART_DELAY_THRESH(ca->delay_min>>4))
				ca->found |= HYSTART_DELAY;
		}
		/*
		 * Either one of two conditions are met,
		 * we exit from slow start immediately.
		 */
		if (ca->found & hystart_detect)
			tp->snd_ssthresh = tp->snd_cwnd;
	}
}
 
/* Track delayed acknowledgment ratio using sliding window
 * ratio = (15*ratio + sample) / 16
 */
//跟踪延迟确认在滑动窗口的比例，主要考虑正常和丢包的时候，是一个参考值
//根据下面这个函数。化简得 delayed_ack = 15/16*delayed_ack + cnt;
//由于 ratio = delayed_ack/ 16;  16*ration = 15*ration+cnt
//所以 ratio = 15/16 * ratio + cnt / 16;
static void bictcp_acked(struct sock *sk, u32 cnt, s32 rtt_us)
{
	const struct inet_connection_sock *icsk = inet_csk(sk);
	const struct tcp_sock *tp = tcp_sk(sk);
	struct bictcp *ca = inet_csk_ca(sk);
	u32 delay;
 
	if (icsk->icsk_ca_state == TCP_CA_Open) {
		cnt -= ca->delayed_ack >> ACK_RATIO_SHIFT;
		ca->delayed_ack += cnt;
	}
 
	/* Some calls are for duplicates without timetamps */
	if (rtt_us < 0)
		return;
	/* Discard delay samples right after fast recovery */
	if ((s32)(tcp_time_stamp - ca->epoch_start) < HZ)
		return;
 
	delay = usecs_to_jiffies(rtt_us) << 3;
	if (delay == 0)
		delay = 1;
 
	/* first time call or link delay decreases */
	if (ca->delay_min == 0 || ca->delay_min > delay)
		ca->delay_min = delay;
 
	/* hystart triggers when cwnd is larger than some threshold */
	if (hystart && tp->snd_cwnd <= tp->snd_ssthresh &&
	    tp->snd_cwnd >= hystart_low_window)
		hystart_update(sk, delay);
}
 
static struct tcp_congestion_ops cubictcp = {
	.init		= bictcp_init,
	.ssthresh	= bictcp_recalc_ssthresh,
	.cong_avoid	= bictcp_cong_avoid,
	.set_state	= bictcp_state,
	.undo_cwnd	= bictcp_undo_cwnd,
	.pkts_acked     = bictcp_acked,
	.owner		= THIS_MODULE,
	.name		= "cubic",
};
 
static int __init cubictcp_register(void)
{
	BUILD_BUG_ON(sizeof(struct bictcp) > ICSK_CA_PRIV_SIZE);
 
	/* Precompute a bunch of the scaling factors that are used per-packet
	 * based on SRTT of 100ms
	 */
        //beta_scale == 8*(1024 + 717) / 3 / (1024 -717 )，大约为15
	beta_scale = 8*(BICTCP_BETA_SCALE+beta)/ 3 / (BICTCP_BETA_SCALE - beta);
        //cube_rtt_scale cube_rtt_scale是bic_scale/RTT, 这里rtt=100ms=0.1s
	cube_rtt_scale = (bic_scale * 10);	/* 1024*c/rtt */
 
	/* calculate the "K" for (wmax-cwnd) = c/rtt * K^3
	 *  so K = cubic_root( (wmax-cwnd)*rtt/c )
	 * the unit of K is bictcp_HZ=2^10, not HZ
	 *
	 *  c = bic_scale >> 10
	 *  rtt = 100ms
	 *
	 * the following code has been designed and tested for
	 * cwnd < 1 million packets
	 * RTT < 100 seconds
	 * HZ < 1,000,00  (corresponding to 10 nano-second)
	 */
	/* 1/c * 2^2*bictcp_HZ * srtt */
        //通过bic_K和paper中的公式对比，可以知道cube_factor就是1/（C/RTT)
        //具体需要参考算法实现，以及这里的30是开根号
	cube_factor = 1ull << (10+3*BICTCP_HZ); /* 2^40 */
	/* divide by bic_scale and by constant Srtt (100ms) */
	do_div(cube_factor, bic_scale * 10);
	return tcp_register_congestion_control(&cubictcp);
}
static void __exit cubictcp_unregister(void)
{
	tcp_unregister_congestion_control(&cubictcp);
}
module_init(cubictcp_register);
module_exit(cubictcp_unregister);
MODULE_AUTHOR("Sangtae Ha, Stephen Hemminger");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CUBIC TCP");
MODULE_VERSION("2.3");
 