using System.Collections;
using UnityEngine;
using UnityEngine.AI;

public class EnemyFSM : MonoBehaviour
{
    public bool checkGizmos;
    public enum EnemyState { Patrol, Chase, Attack, Escape }
    private EnemyState currentState;
    private Transform player;
    private EnemyLife enemyLife;

    public Transform[] patrolPoints;
    private int currentPatrolIndex;
    private Vector3[] randomPatrolPoints = new Vector3[2];
    private int currentRandomPatrolIndex = 0;
    private bool usingRandomPatrolPoints = false;

    public float chaseRange = 10f;
    public float attackRange = 2f;
    public float sightRange = 12f;
    public float sightAngle = 90f;
    public LayerMask obstacleLayers;

    public float maxRandomPatrolDistance = 12f;
    public int maxRandomPatrolAttempts = 10;
    public float minWallDistance = 2f;
    public int wallCheckRays = 8;
    public float minDistanceBetweenPatrolPoints = 5f;
    public bool validateNavMeshPaths = true;

    public float fleeDistance = 15f;
    public float fleeUpdateInterval = 1f;
    private float fleeTimer = 0f;

    private NavMeshAgent agent;
    private Vector3 lastKnownPlayerPosition;
    private bool canSeePlayer = false;
    private float lostSightTime = 0f;
    public float timeBeforeNewPatrolPoints = 5f;
    private float originalSpeed;

    private bool isAttacking = false;
    public float attackCooldown = 2f;

    private void Start()
    {
        enemyLife = GetComponent<EnemyLife>();
        player = GameObject.FindGameObjectWithTag("Player").transform;
        agent = GetComponent<NavMeshAgent>();
        currentState = EnemyState.Patrol;
        originalSpeed = agent.speed;

        if (validateNavMeshPaths && patrolPoints.Length > 0)
        {
            ValidatePatrolPoints();
        }

        if (patrolPoints.Length > 0)
        {
            usingRandomPatrolPoints = false;
            MoveToNextPatrolPoint();
        }
        else
        {
            GenerateRandomPatrolPoints();
            usingRandomPatrolPoints = true;
            agent.SetDestination(randomPatrolPoints[currentRandomPatrolIndex]);
        }
    }

    private void Update()
    {
        canSeePlayer = CanSeePlayer();

        if (canSeePlayer)
        {
            lastKnownPlayerPosition = player.position;
            lostSightTime = 0f;
        }
        else if (currentState == EnemyState.Chase)
        {
            lostSightTime += Time.deltaTime;

            if (lostSightTime >= timeBeforeNewPatrolPoints && !usingRandomPatrolPoints)
            {
                GenerateRandomPatrolPoints();
                usingRandomPatrolPoints = true;
            }
        }

        switch (currentState)
        {
            case EnemyState.Patrol:
                Patrol();
                break;
            case EnemyState.Chase:
                Chase();
                break;
            case EnemyState.Attack:
                if (!isAttacking)
                {
                    StartCoroutine(PerformAttack());
                }
                break;
            case EnemyState.Escape:
                Escape();
                break;
        }

        CheckTransitions();
    }

    private IEnumerator WaitForSeconds(float seconds)
    {
        yield return new WaitForSeconds(seconds);
    }

    private bool CanSeePlayer()
    {
        if (Vector3.Distance(transform.position, player.position) > sightRange)
            return false;

        Vector3 directionToPlayer = player.position - transform.position;

        float angle = Vector3.Angle(transform.forward, directionToPlayer);
        if (angle > sightAngle / 2) 
            return false;

        RaycastHit hit;
        if (Physics.Raycast(transform.position, directionToPlayer, out hit, sightRange, obstacleLayers))
        {
            if (hit.transform != player)
                return false;
        }

        return true;
    }

    private void Patrol()
    {
        if (!agent.pathPending && agent.remainingDistance < 0.5f)
        {
            if (usingRandomPatrolPoints)
            {
                currentRandomPatrolIndex = (currentRandomPatrolIndex + 1) % randomPatrolPoints.Length;
                agent.SetDestination(randomPatrolPoints[currentRandomPatrolIndex]);
            }
            else
            {
                MoveToNextPatrolPoint();
            }
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
        if (canSeePlayer)
        {
            agent.SetDestination(player.position);
        }
    }

    private IEnumerator PerformAttack()
    {
        isAttacking = true;

        agent.speed = 0;

        Debug.Log("Atacando al jugador!");

        yield return new WaitForSeconds(attackCooldown);

        agent.speed = originalSpeed;
        isAttacking = false;

        float distanceToPlayer = Vector3.Distance(transform.position, player.position);
        if (distanceToPlayer > attackRange || !canSeePlayer)
        {
            currentState = EnemyState.Chase;
        }
    }

    private void Escape()
    {
        fleeTimer += Time.deltaTime;
        if (fleeTimer >= fleeUpdateInterval || !agent.hasPath)
        {
            fleeTimer = 0f;

            if (canSeePlayer)
            {
                Vector3 fleeDirection = transform.position - player.position;
                fleeDirection.y = 0;

                Vector3 targetPosition = transform.position + fleeDirection.normalized * fleeDistance;

                NavMeshHit hit;
                if (NavMesh.SamplePosition(targetPosition, out hit, fleeDistance, NavMesh.AllAreas))
                {
                    agent.SetDestination(hit.position);
                }
            }
            else
            {
                Vector3 randomDirection = Random.insideUnitSphere * fleeDistance;
                randomDirection.y = 0;
                Vector3 targetPosition = transform.position + randomDirection;

                NavMeshHit hit;
                if (NavMesh.SamplePosition(targetPosition, out hit, fleeDistance, NavMesh.AllAreas))
                {
                    agent.SetDestination(hit.position);
                }
            }
        }
    }

    private void ValidatePatrolPoints()
    {
        for (int i = 0; i < patrolPoints.Length; i++)
        {
            if (patrolPoints[i] == null) continue;

            int nextIndex = (i + 1) % patrolPoints.Length;
            NavMeshPath path = new NavMeshPath();

            bool validPath = NavMesh.CalculatePath(
                patrolPoints[i].position,
                patrolPoints[nextIndex].position,
                NavMesh.AllAreas,
                path
            );
        }
    }

    private void GenerateRandomPatrolPoints()
    {
        int attemptCount = 0;
        int maxAttempts = maxRandomPatrolAttempts * 3;

        randomPatrolPoints[0] = GetRandomPointOnNavMesh(lastKnownPlayerPosition, maxRandomPatrolDistance);

        for (int i = 1; i < randomPatrolPoints.Length; i++)
        {
            Vector3 validPoint = Vector3.zero;
            bool foundValidPoint = false;

            while (!foundValidPoint && attemptCount < maxAttempts)
            {
                attemptCount++;
                Vector3 candidatePoint = GetRandomPointOnNavMesh(lastKnownPlayerPosition, maxRandomPatrolDistance);

                bool isFarEnough = true;
                for (int j = 0; j < i; j++)
                {
                    if (Vector3.Distance(candidatePoint, randomPatrolPoints[j]) < minDistanceBetweenPatrolPoints)
                    {
                        isFarEnough = false;
                        break;
                    }
                }

                if (isFarEnough && validateNavMeshPaths)
                {
                    NavMeshPath path = new NavMeshPath();
                    if (!NavMesh.CalculatePath(randomPatrolPoints[i - 1], candidatePoint, NavMesh.AllAreas, path) ||
                        path.status != NavMeshPathStatus.PathComplete)
                    {
                        isFarEnough = false;
                    }
                }

                if (isFarEnough)
                {
                    validPoint = candidatePoint;
                    foundValidPoint = true;
                }
            }

            if (!foundValidPoint)
            {
                for (int attempt = 0; attempt < 8; attempt++)
                {
                    float angle = attempt * (360f / 8);
                    Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;
                    Vector3 targetPosition = lastKnownPlayerPosition + direction * minDistanceBetweenPatrolPoints * 1.5f;

                    NavMeshHit hit;
                    if (NavMesh.SamplePosition(targetPosition, out hit, maxRandomPatrolDistance, NavMesh.AllAreas))
                    {
                        NavMeshPath path = new NavMeshPath();
                        if (!validateNavMeshPaths ||
                            (NavMesh.CalculatePath(randomPatrolPoints[i - 1], hit.position, NavMesh.AllAreas, path) &&
                             path.status == NavMeshPathStatus.PathComplete))
                        {
                            validPoint = hit.position;
                            foundValidPoint = true;
                            break;
                        }
                    }
                }

                if (!foundValidPoint)
                {
                    validPoint = randomPatrolPoints[i - 1];
                }
            }

            randomPatrolPoints[i] = validPoint;
        }

        currentRandomPatrolIndex = 0;
        agent.SetDestination(randomPatrolPoints[currentRandomPatrolIndex]);
    }

    private Vector3 GetRandomPointOnNavMesh(Vector3 center, float maxDistance)
    {
        for (int i = 0; i < maxRandomPatrolAttempts; i++)
        {
            Vector2 randomPoint2D = Random.insideUnitCircle * maxDistance;
            Vector3 randomPoint = center + new Vector3(randomPoint2D.x, 0, randomPoint2D.y);

            NavMeshHit hit;
            if (NavMesh.SamplePosition(randomPoint, out hit, maxDistance, NavMesh.AllAreas))
            {
                if (IsPointAwayFromWalls(hit.position))
                {
                    Vector3 directionToCenter = center - hit.position;
                    RaycastHit obstacleHit;
                    if (!Physics.Raycast(hit.position, directionToCenter, out obstacleHit, maxDistance, obstacleLayers))
                    {
                        return hit.position;
                    }
                }
            }
        }

        NavMeshHit centerHit;
        if (NavMesh.SamplePosition(center, out centerHit, maxDistance, NavMesh.AllAreas))
        {
            Vector3 safePoint = AdjustPointAwayFromWalls(centerHit.position);
            return safePoint;
        }

        return center;
    }

    private bool IsPointAwayFromWalls(Vector3 point)
    {
        for (int i = 0; i < wallCheckRays; i++)
        {
            float angle = i * (360f / wallCheckRays);
            Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;

            RaycastHit hit;
            if (Physics.Raycast(point, direction, out hit, minWallDistance, obstacleLayers))
            {
                return false;
            }
        }
        return true;
    }

    // Aqui se intenta alejar el punto escogido de la pared para que si esta muy cerca, el enemigo 
    // pueda llegar correctamente al punto
    private Vector3 AdjustPointAwayFromWalls(Vector3 point)
    {
        Vector3 adjustedPoint = point;
        Vector3 pushDirection = Vector3.zero;
        bool wallDetected = false;

        for (int i = 0; i < wallCheckRays; i++)
        {
            float angle = i * (360f / wallCheckRays);
            Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;

            RaycastHit hit;
            if (Physics.Raycast(point, direction, out hit, minWallDistance * 1.5f, obstacleLayers))
            {
                float pushStrength = minWallDistance - hit.distance;
                pushDirection += -direction.normalized * pushStrength;
                wallDetected = true;
            }
        }

        if (wallDetected)
        {
            adjustedPoint += pushDirection.normalized * minWallDistance;


            NavMeshHit navHit;
            if (NavMesh.SamplePosition(adjustedPoint, out navHit, minWallDistance, NavMesh.AllAreas))
            {
                return navHit.position;
            }
        }

        return adjustedPoint;
    }

    private void CheckTransitions()
    {
        if (isAttacking && currentState == EnemyState.Attack) return;

        float distanceToPlayer = Vector3.Distance(transform.position, player.position);
        bool isHealthLow = enemyLife.IsLowHealth();

        if (isHealthLow && canSeePlayer && distanceToPlayer <= chaseRange)
        {
            currentState = EnemyState.Escape;
        }

        else if (distanceToPlayer <= attackRange && canSeePlayer && !isHealthLow)
        {
            currentState = EnemyState.Attack;
        }

        else if (distanceToPlayer <= chaseRange && canSeePlayer && !isHealthLow)
        {
            currentState = EnemyState.Chase;
            usingRandomPatrolPoints = false;
        }

        else if (currentState == EnemyState.Chase &&
                 lostSightTime >= timeBeforeNewPatrolPoints &&
                 !canSeePlayer &&
                 !agent.pathPending &&
                 agent.remainingDistance < 0.5f)
        {
            currentState = EnemyState.Patrol;
        }

        else if (currentState == EnemyState.Escape && (!canSeePlayer || distanceToPlayer > chaseRange * 1.5f))
        {
            currentState = EnemyState.Patrol;
        }
    }

    private void OnDrawGizmos()
    {
        if (checkGizmos)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(transform.position, chaseRange);

            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(transform.position, attackRange);

            Gizmos.color = canSeePlayer ? Color.green : Color.blue;
            DrawVisionCone();
        }
    }

    private void DrawVisionCone()
    {
        float halfAngle = sightAngle / 2;
        Vector3 forward = transform.forward;
        Vector3 right = transform.right;

        int lineCount = 10;

        for (int i = 0; i <= lineCount; i++)
        {
            float angle = -halfAngle + (sightAngle * i / lineCount);
            float radians = angle * Mathf.Deg2Rad;

            Vector3 direction = forward * Mathf.Cos(radians) + right * Mathf.Sin(radians);
            direction.Normalize();

            Gizmos.DrawRay(transform.position, direction * sightRange);
        }

        int arcCount = 5;
        for (int i = 1; i <= arcCount; i++)
        {
            float radius = sightRange * i / arcCount;
            Vector3 lastPoint = Vector3.zero;

            for (int j = 0; j <= lineCount; j++)
            {
                float angle = -halfAngle + (sightAngle * j / lineCount);
                float radians = angle * Mathf.Deg2Rad;

                Vector3 direction = forward * Mathf.Cos(radians) + right * Mathf.Sin(radians);
                direction.Normalize();

                Vector3 currentPoint = transform.position + direction * radius;

                if (j > 0)
                {
                    Gizmos.DrawLine(lastPoint, currentPoint);
                }

                lastPoint = currentPoint;
            }
        }
    }
}