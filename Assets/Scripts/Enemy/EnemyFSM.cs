using UnityEngine;
using UnityEngine.AI;

public class EnemyFSM : MonoBehaviour
{
    public enum EnemyState { Patrol, Chase, Attack }
    private EnemyState currentState;

    public Transform[] patrolPoints;
    private int currentPatrolIndex;

    public Transform player;
    public float chaseRange = 10f;
    public float attackRange = 2f;

    private NavMeshAgent agent;

    private void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        currentState = EnemyState.Patrol;
        MoveToNextPatrolPoint();
    }

    private void Update()
    {
        switch (currentState)
        {
            case EnemyState.Patrol:
                Patrol();
                break;
            case EnemyState.Chase:
                Chase();
                break;
            case EnemyState.Attack:
                Attack();
                break;
        }

        CheckTransitions();
    }

    private void Patrol()
    {
        if (!agent.pathPending && agent.remainingDistance < 0.5f)
        {
            MoveToNextPatrolPoint();
        }
    }

    private void MoveToNextPatrolPoint()
    {
        if (patrolPoints.Length == 0) return;

        agent.SetDestination(patrolPoints[currentPatrolIndex].position);
        currentPatrolIndex = (currentPatrolIndex + 1) % patrolPoints.Length;
    }

    private void Chase()
    {
        agent.SetDestination(player.position);
    }

    private void Attack()
    {
        agent.ResetPath(); // Se detiene
        // Aquí puedes agregar la lógica de ataque
        Debug.Log("Atacando al jugador!");
    }

    private void CheckTransitions()
    {
        float distanceToPlayer = Vector3.Distance(transform.position, player.position);

        if (distanceToPlayer <= attackRange)
        {
            currentState = EnemyState.Attack;
        }
        else if (distanceToPlayer <= chaseRange)
        {
            currentState = EnemyState.Chase;
        }
        else
        {
            currentState = EnemyState.Patrol;
        }
    }
}
