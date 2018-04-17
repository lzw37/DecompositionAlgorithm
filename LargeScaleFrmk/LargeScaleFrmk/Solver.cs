using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace LargeScaleFrmk
{
    /// <summary>
    /// 求解算法抽象类
    /// </summary>
    public abstract class SolutionAlgorithm
    {
        /// <summary>
        /// 数据集
        /// </summary>
        public virtual DataStructure Data { get; set; }

        /// <summary>
        /// 求解方法
        /// </summary>
        /// <returns></returns>
        public virtual bool Optimize()
        {
            return false;
        }

        /// <summary>
        /// 输出结果
        /// </summary>
        public virtual void Output()
        {

        }
    }
}
