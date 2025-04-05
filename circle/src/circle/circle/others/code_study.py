# 27. Remove Element

# nums = [0,1,2,2,3,0,4,2]
# val = 2

# # [0,1,4,0,3]

# class Solution:
#     def removeElement(self, nums: list[int], val: int) -> int:
#         # count = len(nums)
#         # i = 0 
#         # while count > 0 : 
#         #     if nums[i] == val : 
#         #         nums.pop(i)
#         #         i-=1
#         #     count-=1
#         #     i+=1
#         insert_idx = 0
#         for i in range(len(nums)):
#             if nums[i] != val:
#                 nums[insert_idx] = nums[i]
#                 insert_idx += 1
#         return insert_idx
    
# solution = Solution()
# solution.removeElement(nums, val)
# print(nums)

# 73. Set Matrix Zeroes

# # Input: 
# matrix = [[1,1,1],[1,0,1],[1,1,1]]
# # Output: [[1,0,1],[0,0,0],[1,0,1]]

# class Solution:
#     def setZeroes(self, matrix: list[list[int]]) -> None:
#         """
#         Do not return anything, modify matrix in-place instead.
#         """
#         rows = len(matrix)
#         cols = len(matrix[0])
#         # use set to avoid duplicate
#         zero_rows = set()
#         zero_cols = set()
        
#         for i in range(rows):
#             for j in range(cols):
#                 if matrix[i][j] == 0:
#                     # add row and col to set
#                     zero_rows.add(i)
#                     zero_cols.add(j)

#         for i in range(rows):
#             for j in range(cols):
#                 if i in zero_rows or j in zero_cols:
#                     matrix[i][j] = 0

# solution = Solution()
# solution.setZeroes(matrix)


# 45. Jump Game II

# You are given a 0-indexed array of integers nums of length n. You are initially positioned at nums[0].

# Each element nums[i] represents the maximum length of a forward jump from index i. In other words, if you are at nums[i], you can jump to any nums[i + j] where:

# 0 <= j <= nums[i] and
# i + j < n
# Return the minimum number of jumps to reach nums[n - 1]. The test cases are generated such that you can reach nums[n - 1].

# Example 1:

# Input: 
nums = [2,3,1,1,4]
# Output: 2
# Explanation: The minimum number of jumps to reach the last index is 2. Jump 1 step from index 0 to 1, then 3 steps to the last index.


class Solution:
    def jump(self, nums: list[int]) -> int:
        n = len(nums)
        max_pos = 0
        if n == 1:
            return 0
        jumps = 0
        for i in range(n):
            max_pos = max(max_pos, i + nums[i])
            if i == n-1:
                return jumps
            if i == end:
                end = max_pos
                jumps += 1
        return jumps

solution = Solution()
print(solution.jump(nums))
                
        