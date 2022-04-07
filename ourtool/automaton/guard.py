import re
from typing import List, Dict
from ourtool.automaton.hybrid_io_automaton import HybridIoAutomaton

class LogicTreeNode:
    def __init__(self, data, child = [], val = None, mode_guard = None):
        self.data = data 
        self.child = child
        self.val = val
        self.mode_guard = mode_guard

class Guard:
    def __init__(self, root:LogicTreeNode=None, logic_str:str=None):
        self.logic_tree_root = root
        self.logic_string = logic_str
        if self.logic_tree_root is None and logic_str is not None:
            self.construct_tree_from_str(logic_str)

    def logic_string_split(self, logic_string):
        res = re.split('(and)',logic_string)

        tmp = []
        for sub_str in res:
            tmp += re.split('(or)',sub_str)
        res = tmp

        tmp = []
        for sub_str in res:
            tmp += re.split('(\()',sub_str)
        res = tmp

        tmp = []
        for sub_str in res:
            tmp += re.split('(\))',sub_str)
        res = tmp

        while("" in res) :
            res.remove("")

        for i,sub_str in enumerate(res):
            res[i]= sub_str.strip(' ')
        return res 

    def construct_tree_from_str(self, logic_string:str):
        self.logic_string = logic_string
        logic_string = "(" + logic_string + ")"
        s = self.logic_string_split(logic_string)

        stN = []
        stC = []
        p = {}
        p["and"] = 1
        p["or"] = 1
        p[")"] = 0

        for i in range(len(s)):
            if s[i] == "(":
                stC.append(s[i])
            
            elif s[i] not in p:
                t = LogicTreeNode(s[i])
                stN.append(t)
            
            elif(p[s[i]]>0):
                while (len(stC) != 0 and stC[-1] != '(' and p[stC[-1]] >= p[s[i]]):
                                    # Get and remove the top element
                    # from the character stack
                    t = LogicTreeNode(stC[-1])
                    stC.pop()
    
                    # Get and remove the top element
                    # from the node stack
                    t1 = stN[-1]
                    stN.pop()
    
                    # Get and remove the currently top
                    # element from the node stack
                    t2 = stN[-1]
                    stN.pop()
    
                    # Update the tree
                    t.child = [t1, t2]
    
                    # Push the node to the node stack
                    stN.append(t)
                stC.append(s[i])
            elif (s[i] == ')'):
                while (len(stC) != 0 and stC[-1] != '('):
                    # from the character stack
                    t = LogicTreeNode(stC[-1])
                    stC.pop()
    
                    # Get and remove the top element
                    # from the node stack
                    t1 = stN[-1]
                    stN.pop()
    
                    # Get and remove the currently top
                    # element from the node stack
                    t2 = stN[-1]
                    stN.pop()
    
                    # Update the tree
                    t.child = [t1, t2]
    
                    # Push the node to the node stack
                    stN.append(t)
                stC.pop()
        t = stN[-1]
        self.logic_tree_root = t

    def generate_guard_string(self):
        return self._generate_guard_string(self.logic_tree_root)

    def _generate_guard_string(self, root: LogicTreeNode)->str:
        if root.data!="and" and root.data!="or":
            return root.data
        else:
            data1 = self._generate_guard_string(root.child[0])
            data2 = self._generate_guard_string(root.child[1])
            if root.data == "and":
                return f"And({data1},{data2})"
            elif root.data == "or":
                return f"Or({data1},{data2})"

    def execute_guard(self, aut1:HybridIoAutomaton, aut2:HybridIoAutomaton, discrete_variable_dict:Dict) -> bool:
        # This function will execute guard, and remove guard related to mode from the tree
        # We can do this recursively
        res = self._execute_guard(self.logic_tree_root, aut1, aut2, discrete_variable_dict)
        
        return res

    def _execute_guard(self, root:LogicTreeNode, aut1:HybridIoAutomaton, aut2:HybridIoAutomaton, discrete_variable_dict:Dict) -> bool:
        # If is tree leaf
        if root.child == []:
            # Check if the expression involves mode
            expr = root.data
            is_mode_guard = False
            for key in discrete_variable_dict:
                if key in expr:
                    is_mode_guard = True
                    expr = expr.replace(key, discrete_variable_dict[key])
            if is_mode_guard:
                # Execute guard, assign type and  and return result
                root.mode_guard = True
                res = eval(expr)
                root.val = res 
                return res
            # Otherwise, return True
            else: 
                root.mode_guard = False 
                root.val = True 
                return True
        # For the two children, call _execute_guard and collect result
        res1 = self._execute_guard(root.child[0],aut1,aut2,discrete_variable_dict)
        res2 = self._execute_guard(root.child[1],aut1,aut2,discrete_variable_dict)
        # Evaluate result for current node
        if root.data == "and":
            res = res1 and res2 
        elif root.data == "or":
            res = res1 or res2
        else:
            raise ValueError(f"Invalid root data {root.data}")
        
        # If the result is False, return False
        if not res:
            return False 
        # Else if any child have false result, remove that child
        else: 
            if not res1 or root.child[0].mode_guard:
                root.data = root.child[1].data
                root.val = root.child[1].val 
                root.mode_guard = root.child[1].mode_guard
                root.child = root.child[1].child
            elif not res2 or root.child[1].mode_guard:
                root.data = root.child[0].data
                root.val = root.child[0].val 
                root.mode_guard = root.child[0].mode_guard
                root.child = root.child[0].child
            return True

if __name__ == "__main__":
    tmp = Guard()
    tmp.construct_tree_from_str('other_x-ego_x<20 and other_x-ego_x>10 and other_vehicle_lane==ego_vehicle_lane')
    print("stop")